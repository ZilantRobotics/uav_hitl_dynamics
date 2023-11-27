/*
 * Copyright (c) 2020-2023 RaccoonLab.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#include <iostream>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <algorithm>
#include "vtolDynamicsSim.hpp"
#include <ros/package.h>
#include <array>
#include "cs_converter.hpp"
#include "common_math.hpp"


VtolDynamics::VtolDynamics(){
    state_.angularVel.setZero();
    state_.linearVelNed.setZero();
    state_.airspeedFrd.setZero();
    environment_.windNED.setZero();
    environment_.windVariance = 0;
    params_.accelBias.setZero();
    params_.gyroBias.setZero();
    state_.forces.specific << 0, 0, -params_.gravity;
    for(size_t idx = 0; idx < 8; idx++){
        state_.prevActuators.push_back(0);
        state_.crntActuators.push_back(0);
    }
}

int8_t VtolDynamics::init(){
    loadTables("/uav/aerodynamics_coeffs/");
    loadParams("/uav/aerodynamics_coeffs/");
    return 0;
}

template<int ROWS, int COLS, int ORDER>
Eigen::MatrixXd getTableNew(const std::string& path, const char* name){
    std::vector<double> data;

    if(ros::param::get(path + name, data) == false){
        throw std::invalid_argument(std::string("Wrong parameter name: ") + name);
    }

    return Eigen::Matrix<double, ROWS, COLS, ORDER>(data.data());
}


void VtolDynamics::loadTables(const std::string& path){
    tables_.CS_rudder = getTableNew<8, 20, Eigen::RowMajor>(path, "CS_rudder_table");
    tables_.CS_beta = getTableNew<8, 90, Eigen::RowMajor>(path, "CS_beta");
    tables_.AoA = getTableNew<1, 47, Eigen::RowMajor>(path, "AoA");
    tables_.AoS = getTableNew<90, 1, Eigen::ColMajor>(path, "AoS");
    tables_.actuator = getTableNew<20, 1, Eigen::ColMajor>(path, "actuator_table");
    tables_.airspeed = getTableNew<8, 1, Eigen::ColMajor>(path, "airspeed_table");
    tables_.CLPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CLPolynomial");
    tables_.CSPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CSPolynomial");
    tables_.CDPolynomial = getTableNew<8, 6, Eigen::RowMajor>(path, "CDPolynomial");
    tables_.CmxPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CmxPolynomial");
    tables_.CmyPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CmyPolynomial");
    tables_.CmzPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CmzPolynomial");
    tables_.CmxAileron = getTableNew<8, 20, Eigen::RowMajor>(path, "CmxAileron");
    tables_.CmyElevator = getTableNew<8, 20, Eigen::RowMajor>(path, "CmyElevator");
    tables_.CmzRudder = getTableNew<8, 20, Eigen::RowMajor>(path, "CmzRudder");
    tables_.prop = getTableNew<40, 5, Eigen::RowMajor>(path, "prop");
    if(ros::param::get(path + "actuatorTimeConstants", tables_.actuatorTimeConstants) == false){
        throw std::invalid_argument(std::string("Wrong parameter name: ") + "actuatorTimeConstants");
    }
}

void VtolDynamics::loadParams(const std::string& path){
    double propLocX;
    double propLocY;
    double propLocZ;
    double mainEngineLocX;

    if(!ros::param::get(path + "mass", params_.mass) ||
        !ros::param::get(path + "gravity", params_.gravity) ||
        !ros::param::get(path + "atmoRho", params_.atmoRho) ||
        !ros::param::get(path + "wingArea", params_.wingArea) ||
        !ros::param::get(path + "characteristicLength", params_.characteristicLength) ||
        !ros::param::get(path + "propellersLocationX", propLocX) ||
        !ros::param::get(path + "propellersLocationY", propLocY) ||
        !ros::param::get(path + "propellersLocationZ", propLocZ) ||
        !ros::param::get(path + "mainEngineLocationX", mainEngineLocX) ||
        !ros::param::get(path + "actuatorMin", params_.actuatorMin) ||
        !ros::param::get(path + "actuatorMax", params_.actuatorMax) ||
        !ros::param::get(path + "accVariance", params_.accVariance) ||
        !ros::param::get(path + "gyroVariance", params_.gyroVariance)) {
        // error
    }

    params_.propellersLocation[0] <<  propLocX,  propLocY, propLocZ;
    params_.propellersLocation[1] << -propLocX, -propLocY, propLocZ;
    params_.propellersLocation[2] <<  propLocX, -propLocY, propLocZ;
    params_.propellersLocation[3] << -propLocX,  propLocY, propLocZ;
    params_.propellersLocation[4] << mainEngineLocX, 0, 0;
    params_.inertia = getTableNew<3, 3, Eigen::RowMajor>(path, "inertia");
}

void VtolDynamics::setInitialPosition(const Eigen::Vector3d & position,
                                             const Eigen::Quaterniond& attitudeXYZW){
    state_.position = position;
    state_.attitude = attitudeXYZW;
    state_.initialPose = position;
    state_.initialAttitude = attitudeXYZW;
}
void VtolDynamics::setInitialVelocity(const Eigen::Vector3d & linearVelocity,
                                         const Eigen::Vector3d& angularVelocity){
    state_.linearVelNed = linearVelocity;
    state_.angularVel = angularVelocity;
}

void VtolDynamics::land(){
    state_.forces.specific << 0, 0, -params_.gravity;
    state_.linearVelNed.setZero();
    state_.position[2] = 0.00;

    // Keep previous yaw, but set roll and pitch to 0.0
    state_.attitude.coeffs()[0] = 0;
    state_.attitude.coeffs()[1] = 0;
    state_.attitude.normalize();

    state_.angularVel.setZero();

    std::fill(std::begin(state_.motorsRpm), std::end(state_.motorsRpm), 0.0);
}

int8_t VtolDynamics::calibrate(SimMode_t calType){
    constexpr double MAG_ROTATION_SPEED = 2 * 3.1415 / 10;
    static SimMode_t prevCalibrationType = SimMode_t::NORMAL;
    state_.linearVelNed.setZero();
    state_.position[2] = 0.00;

    switch(calType) {
        case SimMode_t::NORMAL:
            state_.attitude = Eigen::Quaterniond(1, 0, 0, 0);
            state_.angularVel.setZero();
            break;
        case SimMode_t::MAG_1_NORMAL:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(1, 0, 0, 0);
            }
            state_.angularVel << 0.000, 0.000, -MAG_ROTATION_SPEED;
            break;
        case SimMode_t::MAG_2_OVERTURNED:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0, 1, 0, 0);
            }
            state_.angularVel << 0.000, 0.000, MAG_ROTATION_SPEED;
            break;
        case SimMode_t::MAG_3_HEAD_DOWN:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0.707, 0, -0.707, 0);
            }
            state_.angularVel << -MAG_ROTATION_SPEED, 0.000, 0.000;
            break;
        case SimMode_t::MAG_4_HEAD_UP:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0.707, 0, 0.707, 0);
            }
            state_.angularVel << MAG_ROTATION_SPEED, 0.000, 0.000;
            break;
        case SimMode_t::MAG_5_TURNED_LEFT:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0.707, -0.707, 0, 0);
            }
            state_.angularVel << 0.000, MAG_ROTATION_SPEED, 0.000;
            break;
        case SimMode_t::MAG_6_TURNED_RIGHT:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0.707, 0.707, 0, 0);
            }
            state_.angularVel << 0.000, -MAG_ROTATION_SPEED, 0.000;
            break;
        case SimMode_t::MAG_7_ARDUPILOT:
            state_.angularVel << MAG_ROTATION_SPEED, MAG_ROTATION_SPEED, MAG_ROTATION_SPEED;
            break;
        case SimMode_t::MAG_8_ARDUPILOT:
            state_.angularVel << -MAG_ROTATION_SPEED, MAG_ROTATION_SPEED, MAG_ROTATION_SPEED;
            break;
        case SimMode_t::MAG_9_ARDUPILOT:
            state_.angularVel << MAG_ROTATION_SPEED, -MAG_ROTATION_SPEED, MAG_ROTATION_SPEED;
            break;

        case SimMode_t::ACC_1_NORMAL:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(1, 0, 0, 0);
            }
            state_.angularVel.setZero();
            break;
        case SimMode_t::ACC_2_OVERTURNED:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0, 1, 0, 0);
            }
            state_.angularVel.setZero();
            break;
        case SimMode_t::ACC_3_HEAD_DOWN:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0.707, 0, -0.707, 0);
            }
            state_.angularVel.setZero();
            break;
        case SimMode_t::ACC_4_HEAD_UP:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0.707, 0, 0.707, 0);
            }
            state_.angularVel.setZero();
            break;
        case SimMode_t::ACC_5_TURNED_LEFT:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0.707, -0.707, 0, 0);
            }
            state_.angularVel.setZero();
            break;
        case SimMode_t::ACC_6_TURNED_RIGHT:
            if(prevCalibrationType != calType){
                state_.attitude = Eigen::Quaterniond(0.707, 0.707, 0, 0);
            }
            state_.angularVel.setZero();
            break;
        case SimMode_t::AIRSPEED:
            state_.attitude = Eigen::Quaterniond(1, 0, 0, 0);
            state_.angularVel.setZero();
            state_.linearVelNed[0] = 10.0;
            state_.linearVelNed[1] = 10.0;
            break;
        default:
            break;
    }

    auto modeInteger = static_cast<int>(calType);
    if(prevCalibrationType != calType){
        ROS_WARN_STREAM_THROTTLE(1, "init cal " << modeInteger);
        prevCalibrationType = calType;
    }else{
        ROS_WARN_STREAM_THROTTLE(1, "cal " << modeInteger);
    }

    constexpr double DELTA_TIME = 0.001;

    state_.forces.specific = calculateNormalForceWithoutMass();
    Eigen::Quaterniond quaternion(0, state_.angularVel(0), state_.angularVel(1), state_.angularVel(2));
    Eigen::Quaterniond attitudeDelta = state_.attitude * quaternion;
    state_.attitude.coeffs() += attitudeDelta.coeffs() * 0.5 * DELTA_TIME;
    state_.attitude.normalize();
    return 1;
}

void VtolDynamics::process(double dtSecs,
                              const std::vector<double>& motorCmd,
                              bool isCmdPercent){
    Eigen::Vector3d windNed = calculateWind();
    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    state_.airspeedFrd = calculateAirSpeed(rotationMatrix, state_.linearVelNed, windNed);
    double AoA = calculateAnglesOfAtack(state_.airspeedFrd);
    double AoS = calculateAnglesOfSideslip(state_.airspeedFrd);
    auto actuators = isCmdPercent ? mapGeneralCmdToInternal(motorCmd) : motorCmd;
    updateActuators(actuators, dtSecs);
    calculateAerodynamics(state_.airspeedFrd, AoA, AoS, actuators[5], actuators[6], actuators[7],
                          state_.forces.aero, state_.moments.aero);
    calculateNewState(state_.moments.aero, state_.forces.aero, actuators, dtSecs);
}


/**
 * @note Map motors indexes from InnoVTOL mixer into internal representation
 * @param cmd General VTOL actuator command is:
 * 0-3  Copter
 * 4    Ailerons    [ 0.0, +1.0], where 0 wants to rotate to the right
 * 5    Elevators   [-1.0, +1.0], where -1 wants ...
 * 6    Rudders     [-1.0, +1.0], where -1 wants ...
 * 7    Throttle    [0.0,  +1.0]
 * @return Output indexes will be:
 * 0-3  Copter      [0.0,  RAD_PER_SEC_MAX]
 * 4    Throttle    [0.0,  RAD_PER_SEC_MAX]
 * 5    Ailerons    [-1.0, +1.0]
 * 6    Elevators   [-1.0, +1.0]
 * 7    Rudders     [-1.0, +1.0]
 */
std::vector<double> VtolDynamics::mapGeneralCmdToInternal(const std::vector<double>& cmd) const{
    if(cmd.size() < 8){
        std::cerr << "ERROR: VtolDynamics wrong control size. It is " << cmd.size()
                  << ", but should be 8" << std::endl;
        return cmd;
    }

    std::vector<double> actuators(8);
    actuators[0] = cmd[0];
    actuators[1] = cmd[1];
    actuators[2] = cmd[2];
    actuators[3] = cmd[3];

    actuators[4] = cmd[7];      // ICE
    actuators[5] = cmd[4];      // ailerons
    actuators[6] = cmd[5];      // elevators
    actuators[7] = cmd[6];      // rudders

    for(size_t idx = 0; idx < 5; idx++){
        actuators[idx] = boost::algorithm::clamp(actuators[idx], 0.0, +1.0);
        actuators[idx] *= params_.actuatorMax[idx];
    }

    for(size_t idx = 5; idx < 8; idx++){
        actuators[idx] = boost::algorithm::clamp(actuators[idx], -1.0, +1.0);
        actuators[idx] *= (actuators[idx] >= 0) ? params_.actuatorMax[idx] : -params_.actuatorMin[idx];
    }

    return actuators;
}

void VtolDynamics::updateActuators(std::vector<double>& cmd, double dtSecs){
    state_.prevActuators = state_.crntActuators;
    for(size_t idx = 0; idx < 8; idx++){
        auto cmd_delta = state_.prevActuators[idx] - cmd[idx];
        cmd[idx] += cmd_delta * (1 - pow(2.71, -dtSecs/tables_.actuatorTimeConstants[idx]));
        state_.crntActuators[idx] = cmd[idx];
    }
}

Eigen::Vector3d VtolDynamics::calculateWind(){
    Eigen::Vector3d wind;
    wind[0] = sqrt(environment_.windVariance) * distribution_(generator_) + environment_.windNED[0];
    wind[1] = sqrt(environment_.windVariance) * distribution_(generator_) + environment_.windNED[1];
    wind[2] = sqrt(environment_.windVariance) * distribution_(generator_) + environment_.windNED[2];

    /**
     * @note Implement own gust logic
     * innopolis_vtol_indi logic doesn't suit us
     */
    Eigen::Vector3d gust;
    gust.setZero();

    return wind + gust;
}

Eigen::Matrix3d VtolDynamics::calculateRotationMatrix() const{
    return state_.attitude.toRotationMatrix().transpose();
}

Eigen::Vector3d VtolDynamics::calculateAirSpeed(const Eigen::Matrix3d& rotationMatrix,
                                                       const Eigen::Vector3d& velocityNED,
                                                       const Eigen::Vector3d& windSpeedNED) const{
    Eigen::Vector3d airspeedFrd = rotationMatrix * (velocityNED + windSpeedNED);
    if(abs(airspeedFrd[0]) > 40 || abs(airspeedFrd[1]) > 40 || abs(airspeedFrd[2]) > 40){
        airspeedFrd[0] = boost::algorithm::clamp(airspeedFrd[0], -40, +40);
        airspeedFrd[1] = boost::algorithm::clamp(airspeedFrd[1], -40, +40);
        airspeedFrd[2] = boost::algorithm::clamp(airspeedFrd[2], -40, +40);
        std::cout << "Warning: airspeed is out of limit." << std::endl;
    }

    return airspeedFrd;
}

double VtolDynamics::calculateDynamicPressure(double airspeed_mod) const{
    return params_.atmoRho * airspeed_mod * airspeed_mod * params_.wingArea;
}

/**
 * @return AoA
 * it must be [0, 3.14] if angle is [0, +180]
 * it must be [0, -3.14] if angle is [0, -180]
 */
double VtolDynamics::calculateAnglesOfAtack(const Eigen::Vector3d& airspeed_frd) const{
    double A = sqrt(airspeed_frd[0] * airspeed_frd[0] + airspeed_frd[2] * airspeed_frd[2]);
    if(A < 0.001){
        return 0;
    }
    A = airspeed_frd[2] / A;
    A = boost::algorithm::clamp(A, -1.0, +1.0);
    A = (airspeed_frd[0] > 0) ? asin(A) : 3.1415 - asin(A);
    return (A > 3.1415) ? A - 2 * 3.1415 : A;
}

double VtolDynamics::calculateAnglesOfSideslip(const Eigen::Vector3d& airspeed_frd) const{
    double B = airspeed_frd.norm();
    if(B < 0.001){
        return 0;
    }
    B = airspeed_frd[1] / B;
    B = boost::algorithm::clamp(B, -1.0, +1.0);
    return asin(B);
}

/**
 * @note definitions:
 * FD, CD - drug force and drug coeeficient respectively
 * FL - lift force and lift coeeficient respectively
 * FS - side force and side coeeficient respectively
 */
void VtolDynamics::calculateAerodynamics(const Eigen::Vector3d& airspeed,
                                            double AoA,
                                            double AoS,
                                            double aileron_pos,
                                            double elevator_pos,
                                            double rudder_pos,
                                            Eigen::Vector3d& Faero,
                                            Eigen::Vector3d& Maero){
    // 0. Common computation
    double AoA_deg = boost::algorithm::clamp(AoA * 180 / 3.1415, -45.0, +45.0);
    double AoS_deg = boost::algorithm::clamp(AoS * 180 / 3.1415, -90.0, +90.0);
    double airspeedMod = airspeed.norm();
    double dynamicPressure = calculateDynamicPressure(airspeedMod);
    double airspeedModClamped = boost::algorithm::clamp(airspeed.norm(), 5, 40);

    // 1. Calculate aero force
    Eigen::VectorXd polynomialCoeffs(7);
    Eigen::Vector3d FL;
    Eigen::Vector3d FS;
    Eigen::Vector3d FD;

    calculateCLPolynomial(airspeedModClamped, polynomialCoeffs);
    double CL = Math::polyval(polynomialCoeffs, AoA_deg);
    FL = (Eigen::Vector3d(0, 1, 0).cross(airspeed.normalized())) * CL;

    calculateCSPolynomial(airspeedModClamped, polynomialCoeffs);
    double CS = Math::polyval(polynomialCoeffs, AoA_deg);
    double CS_rudder = calculateCSRudder(rudder_pos, airspeedModClamped);
    double CS_beta = calculateCSBeta(AoS_deg, airspeedModClamped);
    FS = airspeed.cross(Eigen::Vector3d(0, 1, 0).cross(airspeed.normalized())) * (CS + CS_rudder + CS_beta);

    calculateCDPolynomial(airspeedModClamped, polynomialCoeffs);
    double CD = Math::polyval(polynomialCoeffs.block<5, 1>(0, 0), AoA_deg);
    FD = (-1 * airspeed).normalized() * CD;

    Faero = 0.5 * dynamicPressure * (FL + FS + FD);

    // 2. Calculate aero moment
    calculateCmxPolynomial(airspeedModClamped, polynomialCoeffs);
    auto Cmx = Math::polyval(polynomialCoeffs, AoA_deg);

    calculateCmyPolynomial(airspeedModClamped, polynomialCoeffs);
    auto Cmy = Math::polyval(polynomialCoeffs, AoA_deg);

    calculateCmzPolynomial(airspeedModClamped, polynomialCoeffs);
    auto Cmz = -Math::polyval(polynomialCoeffs, AoA_deg);

    double Cmx_aileron = calculateCmxAileron(aileron_pos, airspeedModClamped);
    /**
     * @note InnoDynamics from octave has some mistake in elevator logic
     * It always generate non positive moment in both positive and negative position
     * Temporary decision is to create positive moment in positive position and
     * negative moment in negative position
     */
    double Cmy_elevator = calculateCmyElevator(abs(elevator_pos), airspeedModClamped);
    double Cmz_rudder = calculateCmzRudder(rudder_pos, airspeedModClamped);

    auto Mx = Cmx + Cmx_aileron * aileron_pos;
    auto My = Cmy + Cmy_elevator * elevator_pos;
    auto Mz = Cmz + Cmz_rudder * rudder_pos;

    Maero = 0.5 * dynamicPressure * params_.characteristicLength * Eigen::Vector3d(Mx, My, Mz);


    state_.forces.lift << 0.5 * dynamicPressure * params_.characteristicLength * FL;
    state_.forces.drug << 0.5 * dynamicPressure * params_.characteristicLength * FD;
    state_.forces.side << 0.5 * dynamicPressure * params_.characteristicLength * FS;
    state_.moments.steer << Cmx_aileron * aileron_pos, Cmy_elevator * elevator_pos, Cmz_rudder * rudder_pos;
    state_.moments.steer *= 0.5 * dynamicPressure * params_.characteristicLength;
    state_.moments.airspeed << Cmx, Cmy, Cmz;
    state_.moments.airspeed *= 0.5 * dynamicPressure * params_.characteristicLength;
}

void VtolDynamics::thruster(double actuator,
                                   double& thrust, double& torque, double& rpm) const{
    constexpr size_t CONTROL_IDX = 0;
    constexpr size_t THRUST_IDX = 1;
    constexpr size_t TORQUE_IDX = 2;
    constexpr size_t RPM_IDX = 4;

    size_t prev_idx = Math::findPrevRowIdxInMonotonicSequence(tables_.prop, actuator);
    size_t next_idx = prev_idx + 1;
    if(next_idx < tables_.prop.rows()){
        auto prev_row = tables_.prop.row(prev_idx);
        auto next_row = tables_.prop.row(next_idx);
        auto t = (actuator - prev_row(CONTROL_IDX)) / (next_row(CONTROL_IDX) - prev_row(CONTROL_IDX));
        thrust = Math::lerp(prev_row(THRUST_IDX), next_row(THRUST_IDX), t);
        torque = Math::lerp(prev_row(TORQUE_IDX), next_row(TORQUE_IDX), t);
        rpm = Math::lerp(prev_row(RPM_IDX), next_row(RPM_IDX), t);
    }
}

void VtolDynamics::calculateNewState(const Eigen::Vector3d& Maero,
                                        const Eigen::Vector3d& Faero,
                                        const std::vector<double>& actuator,
                                        double dt_sec){
    Eigen::VectorXd thrust(5);
    Eigen::VectorXd torque(5);
    for(size_t idx = 0; idx < 5; idx++){
        thruster(actuator[idx], thrust[idx], torque[idx], state_.motorsRpm[idx]);
    }

    for(size_t idx = 0; idx < 4; idx++){
        state_.forces.motors[idx] << 0, 0, -thrust[idx];
    }
    state_.forces.motors[4] << thrust[4], 0, 0;

    std::array<Eigen::Vector3d, 5> motorTorquesInBodyCS;
    motorTorquesInBodyCS[0] << 0, 0, torque[0];
    motorTorquesInBodyCS[1] << 0, 0, torque[1];
    motorTorquesInBodyCS[2] << 0, 0, -torque[2];
    motorTorquesInBodyCS[3] << 0, 0, -torque[3];
    motorTorquesInBodyCS[4] << -torque[4], 0, 0;
    std::array<Eigen::Vector3d, 5> MdueToArmOfForceInBodyCS;
    for(size_t idx = 0; idx < 5; idx++){
        MdueToArmOfForceInBodyCS[idx] = params_.propellersLocation[idx].cross(state_.forces.motors[idx]);
        state_.moments.motors[idx] = motorTorquesInBodyCS[idx] + MdueToArmOfForceInBodyCS[idx];
    }

    auto MtotalInBodyCS = std::accumulate(&state_.moments.motors[0], &state_.moments.motors[5], Maero);
    state_.angularAccel = calculateAngularAccel(params_.inertia, MtotalInBodyCS, state_.angularVel);
    state_.angularVel += state_.angularAccel * dt_sec;
    Eigen::Quaterniond quaternion(0, state_.angularVel(0), state_.angularVel(1), state_.angularVel(2));
    Eigen::Quaterniond attitudeDelta = state_.attitude * quaternion;
    state_.attitude.coeffs() += attitudeDelta.coeffs() * 0.5 * dt_sec;
    state_.attitude.normalize();

    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    auto& Fmotors = state_.forces.motors;
    Eigen::Vector3d Fspecific = std::accumulate(&Fmotors[0], &Fmotors[5], Faero) / params_.mass;
    Eigen::Vector3d Ftotal = (Fspecific + rotationMatrix * Eigen::Vector3d(0, 0, params_.gravity)) * params_.mass;

    state_.forces.total = Ftotal;
    state_.moments.total = MtotalInBodyCS;

    state_.linearAccel = rotationMatrix.inverse() * Ftotal / params_.mass;
    state_.linearVelNed += state_.linearAccel * dt_sec;
    state_.position += state_.linearVelNed * dt_sec;

    if(state_.position[2] >= 0){
        land();
    }else{
        state_.forces.specific = Fspecific;
    }

    state_.bodylinearVel = rotationMatrix * state_.linearVelNed;
}

Eigen::Vector3d VtolDynamics::calculateNormalForceWithoutMass() const{
    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    return rotationMatrix * Eigen::Vector3d(0, 0, -params_.gravity);
}

void VtolDynamics::calculateCLPolynomial(double airSpeedMod,
                                                Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(tables_.CLPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCSPolynomial(double airSpeedMod,
                                                Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(tables_.CSPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCDPolynomial(double airSpeedMod,
                                                Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(tables_.CDPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCmxPolynomial(double airSpeedMod,
                                                 Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(tables_.CmxPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCmyPolynomial(double airSpeedMod,
                                                 Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(tables_.CmyPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCmzPolynomial(double airSpeedMod,
                                                 Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(tables_.CmzPolynomial, airSpeedMod, polynomialCoeffs);
}
double VtolDynamics::calculateCSRudder(double rudder_pos, double airspeed) const{
    return Math::griddata(-tables_.actuator, tables_.airspeed, tables_.CS_rudder, rudder_pos, airspeed);
}
double VtolDynamics::calculateCSBeta(double AoS_deg, double airspeed) const{
    return Math::griddata(-tables_.AoS, tables_.airspeed, tables_.CS_beta, AoS_deg, airspeed);
}
double VtolDynamics::calculateCmxAileron(double aileron_pos, double airspeed) const{
    return Math::griddata(tables_.actuator, tables_.airspeed, tables_.CmxAileron, aileron_pos, airspeed);
}
double VtolDynamics::calculateCmyElevator(double elevator_pos, double airspeed) const{
    return Math::griddata(tables_.actuator, tables_.airspeed, tables_.CmyElevator, elevator_pos, airspeed);
}
double VtolDynamics::calculateCmzRudder(double rudder_pos, double airspeed) const{
    return Math::griddata(tables_.actuator, tables_.airspeed, tables_.CmzRudder, rudder_pos, airspeed);
}

// Motion dynamics equation
Eigen::Vector3d VtolDynamics::calculateAngularAccel(const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>& inertia,
                                                           const Eigen::Vector3d& moment,
                                                           const Eigen::Vector3d& prevAngVel) const{
    return inertia.inverse() * (moment - prevAngVel.cross(inertia * prevAngVel));
}

/**
 * @note These methods should return in NED format
 */
Eigen::Vector3d VtolDynamics::getVehiclePosition() const{
    return state_.position;
}
Eigen::Vector3d VtolDynamics::getVehicleVelocity() const{
    return state_.linearVelNed;
}
Eigen::Vector3d VtolDynamics::getVehicleAirspeed() const{
    return state_.airspeedFrd;
}

/**
 * @note These methods should return in FRD format
 */
Eigen::Quaterniond VtolDynamics::getVehicleAttitude() const{
    return state_.attitude;
}
Eigen::Vector3d VtolDynamics::getVehicleAngularVelocity() const{
    return state_.angularVel;
}
/**
 * @note We consider that z=0 means ground, so if position <=0, Normal force is appeared,
 * it means that in any way specific force will be equal to Gravity force.
 */
void VtolDynamics::getIMUMeasurement(Eigen::Vector3d& accOutFrd,
                                            Eigen::Vector3d& gyroOutFrd){
    Eigen::Vector3d accNoise(sqrt(params_.accVariance) * distribution_(generator_),
                             sqrt(params_.accVariance) * distribution_(generator_),
                             sqrt(params_.accVariance) * distribution_(generator_));
    Eigen::Vector3d gyroNoise(sqrt(params_.gyroVariance) * distribution_(generator_),
                             sqrt(params_.gyroVariance) * distribution_(generator_),
                             sqrt(params_.gyroVariance) * distribution_(generator_));

    Eigen::Vector3d specificForce(state_.forces.specific);
    Eigen::Vector3d angularVelocity(state_.angularVel);
    Eigen::Quaterniond imuOrient(1, 0, 0, 0);
    accOutFrd = imuOrient.inverse() * specificForce + params_.accelBias + accNoise;
    gyroOutFrd = imuOrient.inverse() * angularVelocity + params_.gyroBias + gyroNoise;
}

/**
 * @note These methods should be private
 */
void VtolDynamics::setWindParameter(Eigen::Vector3d windMeanVelocityNED,
                                       double windVariance){
    environment_.windNED = windMeanVelocityNED;
    environment_.windVariance = windVariance;
}
Eigen::Vector3d VtolDynamics::getAngularAcceleration() const{
    return state_.angularAccel;
}
Eigen::Vector3d VtolDynamics::getLinearAcceleration() const{
    return state_.linearAccel;
}
const Forces& VtolDynamics::getForces() const{
    return state_.forces;
}
const Moments& VtolDynamics::getMoments() const{
    return state_.moments;
}

Eigen::Vector3d VtolDynamics::getBodyLinearVelocity() const{
    return state_.bodylinearVel;
}

bool VtolDynamics::getMotorsRpm(std::vector<double>& motorsRpm) {
    motorsRpm.push_back(state_.motorsRpm[0]);
    motorsRpm.push_back(state_.motorsRpm[1]);
    motorsRpm.push_back(state_.motorsRpm[2]);
    motorsRpm.push_back(state_.motorsRpm[3]);
    motorsRpm.push_back(state_.motorsRpm[4]);
    return true;
}

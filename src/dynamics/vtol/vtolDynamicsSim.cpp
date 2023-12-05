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

static constexpr const size_t MOTORS_AMOUNT = 5;
static constexpr const size_t SERVOS_AMOUNT = 3;
static constexpr const size_t ACTUATORS_AMOUNT = 8;

VtolDynamics::VtolDynamics(){
    _state.angularVel.setZero();
    _state.linearVelNed.setZero();
    _state.airspeedFrd.setZero();
    _environment.windNED.setZero();
    _environment.windVariance = 0;
    _params.accelBias.setZero();
    _params.gyroBias.setZero();
    _state.forces.specific << 0, 0, -_environment.gravity;
    for(size_t idx = 0; idx < ACTUATORS_AMOUNT; idx++){
        _state.prevActuators.push_back(0);
        _state.crntActuators.push_back(0);
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
    _tables.CS_rudder = getTableNew<8, 20, Eigen::RowMajor>(path, "CS_rudder_table");
    _tables.CS_beta = getTableNew<8, 90, Eigen::RowMajor>(path, "CS_beta");
    _tables.AoA = getTableNew<1, 47, Eigen::RowMajor>(path, "AoA");
    _tables.AoS = getTableNew<90, 1, Eigen::ColMajor>(path, "AoS");
    _tables.actuator = getTableNew<20, 1, Eigen::ColMajor>(path, "actuator_table");
    _tables.airspeed = getTableNew<8, 1, Eigen::ColMajor>(path, "airspeed_table");
    _tables.CLPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CLPolynomial");
    _tables.CSPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CSPolynomial");
    _tables.CDPolynomial = getTableNew<8, 6, Eigen::RowMajor>(path, "CDPolynomial");
    _tables.CmxPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CmxPolynomial");
    _tables.CmyPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CmyPolynomial");
    _tables.CmzPolynomial = getTableNew<8, 8, Eigen::RowMajor>(path, "CmzPolynomial");
    _tables.CmxAileron = getTableNew<8, 20, Eigen::RowMajor>(path, "CmxAileron");
    _tables.CmyElevator = getTableNew<8, 20, Eigen::RowMajor>(path, "CmyElevator");
    _tables.CmzRudder = getTableNew<8, 20, Eigen::RowMajor>(path, "CmzRudder");
    _tables.prop = getTableNew<40, 5, Eigen::RowMajor>(path, "prop");
    if(ros::param::get(path + "actuatorTimeConstants", _tables.actuatorTimeConstants) == false){
        throw std::invalid_argument(std::string("Wrong parameter name: ") + "actuatorTimeConstants");
    }
}

void VtolDynamics::loadParams(const std::string& path){
    if(!ros::param::get(path + "mass", _params.mass) ||
        !ros::param::get(path + "gravity", _environment.gravity) ||
        !ros::param::get(path + "atmoRho", _environment.atmoRho) ||
        !ros::param::get(path + "wingArea", _params.wingArea) ||
        !ros::param::get(path + "characteristicLength", _params.characteristicLength) ||

        !ros::param::get(path + "motorMaxSpeed", _params.motorMaxSpeed) ||
        !ros::param::get(path + "servoRange", _params.servoRange) ||

        !ros::param::get(path + "accVariance", _params.accVariance) ||
        !ros::param::get(path + "gyroVariance", _params.gyroVariance)) {
        // error
    }

    loadMotorsGeometry(path);

    _params.inertia = getTableNew<3, 3, Eigen::RowMajor>(path, "inertia");
}

void VtolDynamics::loadMotorsGeometry(const std::string& path) {
    std::vector<double> motorPositionX;
    std::vector<double> motorPositionY;
    std::vector<double> motorPositionZ;
    std::vector<bool> motorDirectionCCW;
    std::vector<double> motorAxisX;
    std::vector<double> motorAxisZ;
    ros::param::get(path + "motorPositionX", motorPositionX);
    ros::param::get(path + "motorPositionY", motorPositionY);
    ros::param::get(path + "motorPositionZ", motorPositionZ);
    ros::param::get(path + "motorDirectionCCW", motorDirectionCCW);
    ros::param::get(path + "motorAxisX", motorAxisX);
    ros::param::get(path + "motorAxisZ", motorAxisZ);

    assert(motorPositionX.size() >= MOTORS_AMOUNT);
    assert(motorPositionY.size() >= MOTORS_AMOUNT);
    assert(motorPositionZ.size() >= MOTORS_AMOUNT);
    assert(motorDirectionCCW.size() >= MOTORS_AMOUNT);
    assert(motorAxisX.size() >= MOTORS_AMOUNT);
    assert(motorAxisZ.size() >= MOTORS_AMOUNT);

    for (size_t motor_idx = 0; motor_idx < motorPositionX.size(); motor_idx++) {
        Geometry geometry;
        geometry.position << motorPositionX[motor_idx], motorPositionY[motor_idx], motorPositionZ[motor_idx];
        geometry.axis << motorAxisX[motor_idx], 0.0, motorAxisZ[motor_idx];
        geometry.directionCCW = motorDirectionCCW[motor_idx];
        _params.geometry.push_back(geometry);
    }
}

void VtolDynamics::setInitialPosition(const Eigen::Vector3d & position,
                                             const Eigen::Quaterniond& attitudeXYZW){
    _state.position = position;
    _state.attitude = attitudeXYZW;
    _state.initialPose = position;
    _state.initialAttitude = attitudeXYZW;
}
void VtolDynamics::setInitialVelocity(const Eigen::Vector3d & linearVelocity,
                                         const Eigen::Vector3d& angularVelocity){
    _state.linearVelNed = linearVelocity;
    _state.angularVel = angularVelocity;
}

void VtolDynamics::land(){
    _state.forces.specific << 0, 0, -_environment.gravity;
    _state.linearVelNed.setZero();
    _state.position[2] = 0.00;

    // Keep previous yaw, but set roll and pitch to 0.0
    _state.attitude.coeffs()[0] = 0;
    _state.attitude.coeffs()[1] = 0;
    _state.attitude.normalize();

    _state.angularVel.setZero();

    std::fill(std::begin(_state.motorsRpm), std::end(_state.motorsRpm), 0.0);
}

int8_t VtolDynamics::calibrate(SimMode_t calType){
    constexpr double MAG_ROTATION_SPEED = 2 * 3.1415 / 10;
    static SimMode_t prevCalibrationType = SimMode_t::NORMAL;
    _state.linearVelNed.setZero();
    _state.position[2] = 0.00;

    switch(calType) {
        case SimMode_t::NORMAL:
            _state.attitude = Eigen::Quaterniond(1, 0, 0, 0);
            _state.angularVel.setZero();
            break;
        case SimMode_t::MAG_1_NORMAL:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(1, 0, 0, 0);
            }
            _state.angularVel << 0.000, 0.000, -MAG_ROTATION_SPEED;
            break;
        case SimMode_t::MAG_2_OVERTURNED:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0, 1, 0, 0);
            }
            _state.angularVel << 0.000, 0.000, MAG_ROTATION_SPEED;
            break;
        case SimMode_t::MAG_3_HEAD_DOWN:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0.707, 0, -0.707, 0);
            }
            _state.angularVel << -MAG_ROTATION_SPEED, 0.000, 0.000;
            break;
        case SimMode_t::MAG_4_HEAD_UP:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0.707, 0, 0.707, 0);
            }
            _state.angularVel << MAG_ROTATION_SPEED, 0.000, 0.000;
            break;
        case SimMode_t::MAG_5_TURNED_LEFT:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0.707, -0.707, 0, 0);
            }
            _state.angularVel << 0.000, MAG_ROTATION_SPEED, 0.000;
            break;
        case SimMode_t::MAG_6_TURNED_RIGHT:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0.707, 0.707, 0, 0);
            }
            _state.angularVel << 0.000, -MAG_ROTATION_SPEED, 0.000;
            break;
        case SimMode_t::MAG_7_ARDUPILOT:
            _state.angularVel << MAG_ROTATION_SPEED, MAG_ROTATION_SPEED, MAG_ROTATION_SPEED;
            break;
        case SimMode_t::MAG_8_ARDUPILOT:
            _state.angularVel << -MAG_ROTATION_SPEED, MAG_ROTATION_SPEED, MAG_ROTATION_SPEED;
            break;
        case SimMode_t::MAG_9_ARDUPILOT:
            _state.angularVel << MAG_ROTATION_SPEED, -MAG_ROTATION_SPEED, MAG_ROTATION_SPEED;
            break;

        case SimMode_t::ACC_1_NORMAL:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(1, 0, 0, 0);
            }
            _state.angularVel.setZero();
            break;
        case SimMode_t::ACC_2_OVERTURNED:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0, 1, 0, 0);
            }
            _state.angularVel.setZero();
            break;
        case SimMode_t::ACC_3_HEAD_DOWN:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0.707, 0, -0.707, 0);
            }
            _state.angularVel.setZero();
            break;
        case SimMode_t::ACC_4_HEAD_UP:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0.707, 0, 0.707, 0);
            }
            _state.angularVel.setZero();
            break;
        case SimMode_t::ACC_5_TURNED_LEFT:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0.707, -0.707, 0, 0);
            }
            _state.angularVel.setZero();
            break;
        case SimMode_t::ACC_6_TURNED_RIGHT:
            if(prevCalibrationType != calType){
                _state.attitude = Eigen::Quaterniond(0.707, 0.707, 0, 0);
            }
            _state.angularVel.setZero();
            break;
        case SimMode_t::AIRSPEED:
            _state.attitude = Eigen::Quaterniond(1, 0, 0, 0);
            _state.angularVel.setZero();
            _state.linearVelNed[0] = 10.0;
            _state.linearVelNed[1] = 10.0;
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

    _state.forces.specific = calculateNormalForceWithoutMass();
    Eigen::Quaterniond quaternion(0, _state.angularVel(0), _state.angularVel(1), _state.angularVel(2));
    Eigen::Quaterniond attitudeDelta = _state.attitude * quaternion;
    _state.attitude.coeffs() += attitudeDelta.coeffs() * 0.5 * DELTA_TIME;
    _state.attitude.normalize();
    return 1;
}

void VtolDynamics::process(double dtSecs, const std::vector<double>& unitless_setpoint){
    _mapUnitlessSetpointToInternal(unitless_setpoint);
    updateActuators(dtSecs);

    Eigen::Vector3d windNed = calculateWind();
    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    _state.airspeedFrd = calculateAirSpeed(rotationMatrix, _state.linearVelNed, windNed);
    double AoA = calculateAnglesOfAtack(_state.airspeedFrd);
    double AoS = calculateAnglesOfSideslip(_state.airspeedFrd);
    calculateAerodynamics(_state.airspeedFrd, AoA, AoS, _servosValues[0], _servosValues[1], _servosValues[2],
                          _state.forces.aero, _state.moments.aero);
    calculateNewState(_state.moments.aero, _state.forces.aero, _motorsRadPerSec, dtSecs);
}


/**
 * @note Map motors indexes from InnoVTOL mixer into internal representation
 * @param cmd General VTOL actuator command is:
 * 0-3  Copter      [ 0.0, +1.0]
 * 4    Ailerons    [-1.0, +1.0]
 * 5    Elevators   [-1.0, +1.0]
 * 6    Rudders     [-1.0, +1.0]
 * 7    Throttle    [0.0,  +1.0]
 * @return Output indexes will be:
 * 0-3  Copter      [0.0,  RAD_PER_SEC_MAX]
 * 4    Throttle    [0.0,  RAD_PER_SEC_MAX]
 * 5    Ailerons    [-MAX_RANGE, +MAX_RANGE]
 * 6    Elevators   [-MAX_RANGE, +MAX_RANGE]
 * 7    Rudders     [-MAX_RANGE, +MAX_RANGE]
 */
void VtolDynamics::_mapUnitlessSetpointToInternal(const std::vector<double>& cmd) {
    assert((cmd.size() >= ACTUATORS_AMOUNT) && "ERROR: VtolDynamics wrong setpoint size.");

    _motorsRadPerSec[0] = cmd[0];
    _motorsRadPerSec[1] = cmd[1];
    _motorsRadPerSec[2] = cmd[2];
    _motorsRadPerSec[3] = cmd[3];
    _motorsRadPerSec[4] = cmd[7];       // ICE

    _servosValues[0] = cmd[4];          // ailerons
    _servosValues[1] = cmd[5];          // elevators
    _servosValues[2] = cmd[6];          // rudders

    for(size_t idx = 0; idx < MOTORS_AMOUNT; idx++){
        _motorsRadPerSec[idx] = boost::algorithm::clamp(_motorsRadPerSec[idx], 0.0, +1.0);
        _motorsRadPerSec[idx] *= _params.motorMaxSpeed[idx];
    }

    for(size_t servo_idx = 0; servo_idx < SERVOS_AMOUNT; servo_idx++){
        size_t idx = servo_idx + MOTORS_AMOUNT;
        _servosValues[servo_idx] = boost::algorithm::clamp(_servosValues[servo_idx], -1.0, +1.0);
        _servosValues[servo_idx] *= _params.servoRange[servo_idx];
    }
}

void VtolDynamics::updateActuators(double dtSecs){
    _state.prevActuators = _state.crntActuators;
    for(size_t idx = 0; idx < MOTORS_AMOUNT; idx++){
        auto cmd_delta = _state.prevActuators[idx] - _motorsRadPerSec[idx];
        _motorsRadPerSec[idx] += cmd_delta * (1 - pow(2.71, -dtSecs/_tables.actuatorTimeConstants[idx]));
        _state.crntActuators[idx] = _motorsRadPerSec[idx];
    }
}

Eigen::Vector3d VtolDynamics::calculateWind(){
    Eigen::Vector3d wind;
    wind[0] = sqrt(_environment.windVariance) * _distribution(_generator) + _environment.windNED[0];
    wind[1] = sqrt(_environment.windVariance) * _distribution(_generator) + _environment.windNED[1];
    wind[2] = sqrt(_environment.windVariance) * _distribution(_generator) + _environment.windNED[2];

    /**
     * @note Implement own gust logic
     * innopolis_vtol_indi logic doesn't suit us
     */
    Eigen::Vector3d gust;
    gust.setZero();

    return wind + gust;
}

Eigen::Matrix3d VtolDynamics::calculateRotationMatrix() const{
    return _state.attitude.toRotationMatrix().transpose();
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
    return _environment.atmoRho * airspeed_mod * airspeed_mod * _params.wingArea;
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

    Maero = 0.5 * dynamicPressure * _params.characteristicLength * Eigen::Vector3d(Mx, My, Mz);


    _state.forces.lift << 0.5 * dynamicPressure * _params.characteristicLength * FL;
    _state.forces.drug << 0.5 * dynamicPressure * _params.characteristicLength * FD;
    _state.forces.side << 0.5 * dynamicPressure * _params.characteristicLength * FS;
    _state.moments.steer << Cmx_aileron * aileron_pos, Cmy_elevator * elevator_pos, Cmz_rudder * rudder_pos;
    _state.moments.steer *= 0.5 * dynamicPressure * _params.characteristicLength;
    _state.moments.airspeed << Cmx, Cmy, Cmz;
    _state.moments.airspeed *= 0.5 * dynamicPressure * _params.characteristicLength;
}

void VtolDynamics::thruster(double actuator,
                            double& thrust, double& torque, double& rpm) const{
    constexpr size_t CONTROL_IDX = 0;
    constexpr size_t THRUST_IDX = 1;
    constexpr size_t TORQUE_IDX = 2;
    constexpr size_t RPM_IDX = 4;

    size_t prev_idx = Math::findPrevRowIdxInMonotonicSequence(_tables.prop, actuator);
    size_t next_idx = prev_idx + 1;
    if(next_idx < _tables.prop.rows()){
        auto prev_row = _tables.prop.row(prev_idx);
        auto next_row = _tables.prop.row(next_idx);
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
    assert((actuator.size() >= MOTORS_AMOUNT) && "ERROR: VtolDynamics wrong motors number.");
    for(size_t idx = 0; idx < MOTORS_AMOUNT; idx++){
        double thrust;
        double torque;
        thruster(actuator[idx], thrust, torque, _state.motorsRpm[idx]);
        _state.forces.motors[idx] = _params.geometry[idx].axis * thrust;

        // Cunterclockwise rotation means positive torque, clockwise - negative
        double ccw = _params.geometry[idx].directionCCW ? 1.0 : -1.0;
        Eigen::Vector3d motorTorquesInBodyCS = _params.geometry[idx].axis * (-1.0) * ccw * torque;

        Eigen::Vector3d MdueToArmOfForceInBodyCS = _params.geometry[idx].position.cross(_state.forces.motors[idx]);
        _state.moments.motors[idx] = motorTorquesInBodyCS + MdueToArmOfForceInBodyCS;
    }

    auto MtotalInBodyCS = std::accumulate(&_state.moments.motors[0], &_state.moments.motors[MOTORS_AMOUNT], Maero);
    _state.angularAccel = calculateAngularAccel(_params.inertia, MtotalInBodyCS, _state.angularVel);
    _state.angularVel += _state.angularAccel * dt_sec;
    Eigen::Quaterniond quaternion(0, _state.angularVel(0), _state.angularVel(1), _state.angularVel(2));
    Eigen::Quaterniond attitudeDelta = _state.attitude * quaternion;
    _state.attitude.coeffs() += attitudeDelta.coeffs() * 0.5 * dt_sec;
    _state.attitude.normalize();

    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    auto& Fmotors = _state.forces.motors;
    Eigen::Vector3d Fspecific = std::accumulate(&Fmotors[0], &Fmotors[MOTORS_AMOUNT], Faero) / _params.mass;
    Eigen::Vector3d Ftotal = (Fspecific + rotationMatrix * Eigen::Vector3d(0, 0, _environment.gravity)) * _params.mass;

    _state.forces.total = Ftotal;
    _state.moments.total = MtotalInBodyCS;

    _state.linearAccel = rotationMatrix.inverse() * Ftotal / _params.mass;
    _state.linearVelNed += _state.linearAccel * dt_sec;
    _state.position += _state.linearVelNed * dt_sec;

    if(_state.position[2] >= 0){
        land();
    }else{
        _state.forces.specific = Fspecific;
    }

    _state.bodylinearVel = rotationMatrix * _state.linearVelNed;
}

Eigen::Vector3d VtolDynamics::calculateNormalForceWithoutMass() const{
    Eigen::Matrix3d rotationMatrix = calculateRotationMatrix();
    return rotationMatrix * Eigen::Vector3d(0, 0, -_environment.gravity);
}

void VtolDynamics::calculateCLPolynomial(double airSpeedMod,
                                                Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(_tables.CLPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCSPolynomial(double airSpeedMod,
                                                Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(_tables.CSPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCDPolynomial(double airSpeedMod,
                                                Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(_tables.CDPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCmxPolynomial(double airSpeedMod,
                                                 Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(_tables.CmxPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCmyPolynomial(double airSpeedMod,
                                                 Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(_tables.CmyPolynomial, airSpeedMod, polynomialCoeffs);
}
void VtolDynamics::calculateCmzPolynomial(double airSpeedMod,
                                                 Eigen::VectorXd& polynomialCoeffs) const{
    Math::calculatePolynomial(_tables.CmzPolynomial, airSpeedMod, polynomialCoeffs);
}
double VtolDynamics::calculateCSRudder(double rudder_pos, double airspeed) const{
    return Math::griddata(-_tables.actuator, _tables.airspeed, _tables.CS_rudder, rudder_pos, airspeed);
}
double VtolDynamics::calculateCSBeta(double AoS_deg, double airspeed) const{
    return Math::griddata(-_tables.AoS, _tables.airspeed, _tables.CS_beta, AoS_deg, airspeed);
}
double VtolDynamics::calculateCmxAileron(double aileron_pos, double airspeed) const{
    return Math::griddata(_tables.actuator, _tables.airspeed, _tables.CmxAileron, aileron_pos, airspeed);
}
double VtolDynamics::calculateCmyElevator(double elevator_pos, double airspeed) const{
    return Math::griddata(_tables.actuator, _tables.airspeed, _tables.CmyElevator, elevator_pos, airspeed);
}
double VtolDynamics::calculateCmzRudder(double rudder_pos, double airspeed) const{
    return Math::griddata(_tables.actuator, _tables.airspeed, _tables.CmzRudder, rudder_pos, airspeed);
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
    return _state.position;
}
Eigen::Vector3d VtolDynamics::getVehicleVelocity() const{
    return _state.linearVelNed;
}
Eigen::Vector3d VtolDynamics::getVehicleAirspeed() const{
    return _state.airspeedFrd;
}

/**
 * @note These methods should return in FRD format
 */
Eigen::Quaterniond VtolDynamics::getVehicleAttitude() const{
    return _state.attitude;
}
Eigen::Vector3d VtolDynamics::getVehicleAngularVelocity() const{
    return _state.angularVel;
}
/**
 * @note We consider that z=0 means ground, so if position <=0, Normal force is appeared,
 * it means that in any way specific force will be equal to Gravity force.
 */
void VtolDynamics::getIMUMeasurement(Eigen::Vector3d& accOutFrd,
                                            Eigen::Vector3d& gyroOutFrd){
    Eigen::Vector3d accNoise(sqrt(_params.accVariance) * _distribution(_generator),
                             sqrt(_params.accVariance) * _distribution(_generator),
                             sqrt(_params.accVariance) * _distribution(_generator));
    Eigen::Vector3d gyroNoise(sqrt(_params.gyroVariance) * _distribution(_generator),
                             sqrt(_params.gyroVariance) * _distribution(_generator),
                             sqrt(_params.gyroVariance) * _distribution(_generator));

    Eigen::Vector3d specificForce(_state.forces.specific);
    Eigen::Vector3d angularVelocity(_state.angularVel);
    Eigen::Quaterniond imuOrient(1, 0, 0, 0);
    accOutFrd = imuOrient.inverse() * specificForce + _params.accelBias + accNoise;
    gyroOutFrd = imuOrient.inverse() * angularVelocity + _params.gyroBias + gyroNoise;
}

/**
 * @note These methods should be private
 */
void VtolDynamics::setWindParameter(Eigen::Vector3d windMeanVelocityNED,
                                       double windVariance){
    _environment.windNED = windMeanVelocityNED;
    _environment.windVariance = windVariance;
}
Eigen::Vector3d VtolDynamics::getAngularAcceleration() const{
    return _state.angularAccel;
}
Eigen::Vector3d VtolDynamics::getLinearAcceleration() const{
    return _state.linearAccel;
}
const Forces& VtolDynamics::getForces() const{
    return _state.forces;
}
const Moments& VtolDynamics::getMoments() const{
    return _state.moments;
}

Eigen::Vector3d VtolDynamics::getBodyLinearVelocity() const{
    return _state.bodylinearVel;
}

bool VtolDynamics::getMotorsRpm(std::vector<double>& motorsRpm) {
    for (auto motor_rpm : _state.motorsRpm) {
        motorsRpm.push_back(motor_rpm);
    }

    return true;
}

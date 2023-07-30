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


#include "octocopter.hpp"
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>



static const std::string MULTICOPTER_PARAMS_NS = "/uav/aerodynamics_coeffs/";
template <class T>
static void getParameter(const std::string& name, T& parameter, T default_value, std::string unit = ""){
  if (!ros::param::get(MULTICOPTER_PARAMS_NS + name, parameter)){
    std::cout << "Did not get "
              << name
              << " from the params, defaulting to "
              << default_value
              << " "
              << unit
              << std::endl;
    parameter = default_value;
  }
}

int8_t MultirotorDynamics::init(){
    // Vehicle parameters
    double vehicleMass;
    double motorTimeconstant;
    double motorRotationalInertia;
    double thrustCoeff;
    double torqueCoeff;
    double dragCoeff;
    getParameter("vehicle_mass",              vehicleMass,                1.,       "kg");
    getParameter("motor_time_constant",       motorTimeconstant,          0.02,     "sec");
    getParameter("motor_rotational_inertia",  motorRotationalInertia,     6.62e-6,  "kg m^2");
    getParameter("thrust_coefficient",        thrustCoeff,                1.91e-6,  "N/(rad/s)^2");
    getParameter("torque_coefficient",        torqueCoeff,                2.6e-7,   "Nm/(rad/s)^2");
    getParameter("drag_coefficient",          dragCoeff,                  0.1,      "N/(m/s)");

    Eigen::Matrix3d aeroMomentCoefficient = Eigen::Matrix3d::Zero();
    getParameter("aeromoment_coefficient_xx", aeroMomentCoefficient(0, 0), 0.003,    "Nm/(rad/s)^2");
    getParameter("aeromoment_coefficient_yy", aeroMomentCoefficient(1, 1), 0.003,    "Nm/(rad/s)^2");
    getParameter("aeromoment_coefficient_zz", aeroMomentCoefficient(2, 2), 0.003,    "Nm/(rad/s)^2");

    Eigen::Matrix3d vehicleInertia = Eigen::Matrix3d::Zero();
    getParameter("vehicle_inertia_xx",        vehicleInertia(0, 0),        0.0049,   "kg m^2");
    getParameter("vehicle_inertia_yy",        vehicleInertia(1, 1),        0.0049,   "kg m^2");
    getParameter("vehicle_inertia_zz",        vehicleInertia(2, 2),        0.0069,   "kg m^2");

    double minPropSpeed = 0.0;
    double maxPropSpeed;
    double momentProcessNoiseAutoCorrelation;
    double forceProcessNoiseAutoCorrelation;
    getParameter("max_prop_speed",            maxPropSpeed,               2200.0,   "rad/s");
    getParameter("moment_process_noise", momentProcessNoiseAutoCorrelation, 1.25e-7,  "(Nm)^2 s");
    getParameter("force_process_noise", forceProcessNoiseAutoCorrelation, 0.0005,   "N^2 s");


    // Set gravity vector according to ROS reference axis system, see header file
    Eigen::Vector3d gravity(0., 0., -9.81);

    double momentArm;
    getParameter("moment_arm",                momentArm,                  0.08,     "m");

    // Create quadcopter simulator
    multicopterSim_ = std::make_unique<MulticopterDynamicsSim>(number_of_motors, thrustCoeff, torqueCoeff,
                        minPropSpeed, maxPropSpeed, motorTimeconstant, motorRotationalInertia,
                        vehicleMass, vehicleInertia,
                        aeroMomentCoefficient, dragCoeff, momentProcessNoiseAutoCorrelation,
                        forceProcessNoiseAutoCorrelation, gravity);

    double initPropSpeed = sqrt(vehicleMass/4.*9.81/thrustCoeff);
    multicopterSim_->setMotorSpeed(initPropSpeed);


    // Get and set IMU parameters
    double accBiasProcessNoiseAutoCorrelation;
    double gyroBiasProcessNoiseAutoCorrelation;
    double accBiasInitVar;
    double gyroBiasInitVar;
    double accMeasNoiseVariance;
    double gyroMeasNoiseVariance;
    getParameter("accelerometer_biasprocess", accBiasProcessNoiseAutoCorrelation, 1.0e-7, "m^2/s^5");
    getParameter("gyroscope_biasprocess",     accBiasProcessNoiseAutoCorrelation, 1.0e-7, "rad^2/s^3");
    getParameter("accelerometer_biasinitvar", accBiasInitVar,                     0.005,  "(m/s^2)^2");
    getParameter("gyroscope_biasinitvar",     gyroBiasInitVar,                    0.003,  "(rad/s)^2");
    getParameter("accelerometer_variance",    accMeasNoiseVariance,               0.005,  "m^2/s^4");
    getParameter("gyroscope_variance",        gyroMeasNoiseVariance,              0.003,  "rad^2/s^2");
    multicopterSim_->imu_.setBias(accBiasInitVar, gyroBiasInitVar,
                                  accBiasProcessNoiseAutoCorrelation, gyroBiasProcessNoiseAutoCorrelation);
    multicopterSim_->imu_.setNoiseVariance(accMeasNoiseVariance, gyroMeasNoiseVariance);

    initStaticMotorTransform(momentArm);

    return 0;
}

void MultirotorDynamics::setInitialPosition(const Eigen::Vector3d & position,
                                               const Eigen::Quaterniond& attitude){
    multicopterSim_->setVehiclePosition(position, attitude);
}

void MultirotorDynamics::process(double dt_secs,
                                    const std::vector<double> & motorSpeedCommandIn,
                                    bool isCmdPercent){
    auto actuators = mapCmdActuator(motorSpeedCommandIn);
    multicopterSim_->proceedState_ExplicitEuler(dt_secs, actuators, isCmdPercent);
}

Eigen::Vector3d MultirotorDynamics::getVehiclePosition() const{
    return multicopterSim_->getVehiclePosition();
}
Eigen::Quaterniond MultirotorDynamics::getVehicleAttitude() const{
    return multicopterSim_->getVehicleAttitude();
}
Eigen::Vector3d MultirotorDynamics::getVehicleVelocity(void) const{
    return multicopterSim_->getVehicleVelocity();
}
Eigen::Vector3d MultirotorDynamics::getVehicleAngularVelocity(void) const{
    return multicopterSim_->getVehicleAngularVelocity();
}
void MultirotorDynamics::getIMUMeasurement(Eigen::Vector3d & accOutput,
                                              Eigen::Vector3d & gyroOutput){
    return multicopterSim_->getIMUMeasurement(accOutput, gyroOutput);
}

bool MultirotorDynamics::getMotorsRpm(std::vector<double>& motorsRpm) {
    const auto motorsSpeed = multicopterSim_->getMotorsSpeed();
    for (auto motorSpeed : motorsSpeed) {
        motorsRpm.push_back(motorSpeed * 9.54929658551);  // rad/sec to RPM
    }

    return true;
}

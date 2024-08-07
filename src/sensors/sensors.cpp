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

#include "sensors.hpp"
#include <cstdlib>
#include <ctime>
#include <boost/algorithm/clamp.hpp>
#include "sensors_isa_model.hpp"
#include "cs_converter.hpp"

Sensors::Sensors(ros::NodeHandle* nh) :
    attitudeSensor(nh,      "/uav/attitude",            0.005),
    pressureSensor(nh,      "/uav/static_pressure",     0.05),
    temperatureSensor(nh,   "/uav/static_temperature",  0.05),
    diffPressureSensor(nh,  "/uav/raw_air_data",        0.05),
    iceStatusSensor(nh,     "/uav/ice",                 0.25),
    imuSensor(nh,           "/uav/imu",                 0.00333),
    velocitySensor_(nh,     "/uav/velocity",            0.05),
    gpsSensor(nh,           "/uav/gps_point",           0.1),
    magSensor(nh,           "/uav/mag",                 0.03),
    escStatusSensor(nh,     "/uav/esc_status",          0.25),
    fuelTankSensor(nh,      "/uav/fuel_tank",           1.0),
    batteryInfoSensor(nh,   "/uav/battery",             1.0)
{
}

int8_t Sensors::init(const std::shared_ptr<UavDynamicsSimBase>& uavDynamicsSim) {
    _uavDynamicsSim = uavDynamicsSim;

    double latRef;
    double lonRef;
    double altRef;
    const std::string SIM_PARAMS_PATH = "/uav/sim_params/";
    bool isEnabled;

    if(!ros::param::get(SIM_PARAMS_PATH + "lat_ref", latRef) ||
       !ros::param::get(SIM_PARAMS_PATH + "lon_ref", lonRef) ||
       !ros::param::get(SIM_PARAMS_PATH + "alt_ref", altRef)){
        ROS_ERROR("Sensors: lat_ref, lon_ref or alt_ref in not present.");
        return -1;
    }

    if (ros::param::get(SIM_PARAMS_PATH + "esc_status", isEnabled) && isEnabled) {
        escStatusSensor.enable();
    }

    if (ros::param::get(SIM_PARAMS_PATH + "ice_status", isEnabled) && isEnabled) {
        iceStatusSensor.enable();
    }

    if (ros::param::get(SIM_PARAMS_PATH + "fuel_tank_status", isEnabled) && isEnabled) {
        fuelTankSensor.enable();
    }

    if (ros::param::get(SIM_PARAMS_PATH + "battery_status", isEnabled) && isEnabled) {
        batteryInfoSensor.enable();
    }

    attitudeSensor.enable();
    imuSensor.enable();
    velocitySensor_.enable();
    magSensor.enable();
    diffPressureSensor.enable();
    temperatureSensor.enable();
    pressureSensor.enable();
    gpsSensor.enable();

    geodeticConverter.setInitialValues(latRef, lonRef, altRef);

    return 0;
}

static const constexpr uint8_t PX4_NED_FRD = 0;
static const constexpr uint8_t ROS_ENU_FLU = 1;

/**
 * @note Different simulators return data in different notation (PX4 or ROS)
 * But we must publish only in PX4 notation
 */
void Sensors::publishStateToCommunicator(uint8_t dynamicsNotation) {
    // 1. Get data from simulator
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    _uavDynamicsSim->getIMUMeasurement(acc, gyro);
    Eigen::Vector3d position = _uavDynamicsSim->getVehiclePosition();
    Eigen::Vector3d linVel = _uavDynamicsSim->getVehicleVelocity();
    auto airspeed = _uavDynamicsSim->getVehicleAirspeed();
    Eigen::Vector3d angVel = _uavDynamicsSim->getVehicleAngularVelocity();
    Eigen::Quaterniond attitude = _uavDynamicsSim->getVehicleAttitude();

    // 2. Convert them to appropriate CS
    Eigen::Vector3d gpsPosition;
    Eigen::Vector3d enuPosition;
    Eigen::Vector3d linVelNed;
    Eigen::Vector3d airspeedFrd;
    Eigen::Vector3d accFrd;
    Eigen::Vector3d gyroFrd;
    Eigen::Vector3d angVelFrd;
    Eigen::Quaterniond attitudeFrdToNed;
    if(dynamicsNotation == PX4_NED_FRD){
        enuPosition = Converter::nedToEnu(position);
        linVelNed = linVel;
        accFrd = acc;
        gyroFrd = gyro;
        angVelFrd = angVel;
        attitudeFrdToNed = attitude;
        airspeedFrd = airspeed;
    }else{
        enuPosition = position;
        linVelNed =  Converter::enuToNed(linVel);
        accFrd = Converter::fluToFrd(acc);
        gyroFrd = Converter::fluToFrd(gyro);
        angVelFrd = Converter::fluToFrd(angVel);
        attitudeFrdToNed = Converter::fluEnuToFrdNed(attitude);
        airspeedFrd = Converter::fluToFrd(airspeed);
    }
    geodeticConverter.enuToGeodetic(enuPosition[0], enuPosition[1], enuPosition[2],
                                    &gpsPosition[0], &gpsPosition[1], &gpsPosition[2]);

    // 3. Calculate temperature, abs pressure and diff pressure using ISA model
    float temperatureKelvin;
    float absPressureHpa;
    float diffPressureHpa;
    SensorModelISA::EstimateAtmosphere(gpsPosition, airspeedFrd,
                                       temperatureKelvin, absPressureHpa, diffPressureHpa);

    // Publish state to communicator
    attitudeSensor.publish(Converter::frdNedTofluEnu(attitudeFrdToNed));
    imuSensor.publish(accFrd, gyroFrd);
    velocitySensor_.publish(linVelNed, angVelFrd);
    magSensor.publish(gpsPosition, attitudeFrdToNed);
    diffPressureSensor.publish(diffPressureHpa);
    pressureSensor.publish(absPressureHpa);
    temperatureSensor.publish(temperatureKelvin);
    gpsSensor.publish(gpsPosition);

    std::vector<double> motorsRpm;
    if(_uavDynamicsSim->getMotorsRpm(motorsRpm)){
        escStatusSensor.publish(motorsRpm);
        if(motorsRpm.size() >= 5){
            iceStatusSensor.publish(motorsRpm[4]);
        }
    }

    static double trueFuelLevelPct = 80.0;
    if(motorsRpm[4] > 0.0) {
        trueFuelLevelPct -= 0.0000002 * motorsRpm[4];
        if(trueFuelLevelPct < 0) {
            trueFuelLevelPct = 0;
        }
    }
    auto fuelNoise = (float)(std::rand() % 26 - 13);
    float measuredFuelLevelPct = boost::algorithm::clamp(trueFuelLevelPct + fuelNoise, 0.0, 100.0);
    fuelTankSensor.publish(measuredFuelLevelPct);

    batteryInfoSensor.publish(1.00f);
}

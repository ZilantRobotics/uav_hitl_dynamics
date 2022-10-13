/* 
 * Copyright (c) 2020-2022 RaccoonLab.
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

Sensors::Sensors(ros::NodeHandle* nh) :
    attitudeSensor(nh,      "/uav/attitude",            0.005),
    imuSensor(nh,           "/uav/imu",                 0.00333),
    velocitySensor_(nh,     "/uav/velocity",            0.05),
    magSensor(nh,           "/uav/mag",                 0.03),
    rawAirDataSensor(nh,    "/uav/raw_air_data",        0.05),
    temperatureSensor(nh,   "/uav/static_temperature",  0.05),
    pressureSensor(nh,      "/uav/static_pressure",     0.05),
    gpsSensor(nh,           "/uav/gps_position",        0.1),
    escStatusSensor(nh,     "/uav/esc_status",          0.25),
    iceStatusSensor(nh,     "/uav/ice_status",          0.25),
    fuelTankSensor(nh,      "/uav/fuel_tank",           2.0),
    batteryInfoSensor(nh,   "/uav/battery",             1.0)
{
}

int8_t Sensors::init() {
    const std::string SIM_PARAMS_PATH = "/uav/sim_params/";
    bool isEnabled;

    attitudeSensor.enable();
    imuSensor.enable();
    velocitySensor_.enable();
    magSensor.enable();
    rawAirDataSensor.enable();
    temperatureSensor.enable();
    pressureSensor.enable();
    gpsSensor.enable();

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

    return 0;
}

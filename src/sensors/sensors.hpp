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

#ifndef SRC_SENSORS_SENSORS_HPP_
#define SRC_SENSORS_SENSORS_HPP_

#include "ice.hpp"

struct Sensors {
    Sensors(ros::NodeHandle* nh);
    int8_t init();

    AttitudeSensor attitudeSensor;
    ImuSensor imuSensor;
    VelocitySensor velocitySensor_;
    MagSensor magSensor;
    RawAirDataSensor rawAirDataSensor;
    TemperatureSensor temperatureSensor;
    PressureSensor pressureSensor;
    GpsSensor gpsSensor;

    EscStatusSensor escStatusSensor;
    IceStatusSensor iceStatusSensor;
    FuelTankSensor fuelTankSensor;
    BatteryInfoSensor batteryInfoSensor;
};

#endif  // SRC_SENSORS_SENSORS_HPP_

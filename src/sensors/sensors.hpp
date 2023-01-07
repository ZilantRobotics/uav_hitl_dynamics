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

#ifndef SRC_SENSORS_SENSORS_HPP_
#define SRC_SENSORS_SENSORS_HPP_

#include "attitude.hpp"
#include "barometer.hpp"
#include "differential_pressure.hpp"
#include "ice.hpp"
#include "uavDynamicsSimBase.hpp"

class Sensors {
public:
    explicit Sensors(ros::NodeHandle* nh);
    int8_t init(UavDynamicsSimBase* uavDynamicsSim);
    void publishStateToCommunicator(uint8_t dynamicsNotation);

    IceStatusSensor iceStatusSensor;

private:
    AttitudeSensor attitudeSensor;
    ImuSensor imuSensor;
    VelocitySensor velocitySensor_;
    MagSensor magSensor;
    DifferentialPressureSensor diffPressureSensor;
    TemperatureSensor temperatureSensor;
    PressureSensor pressureSensor;
    GpsSensor gpsSensor;

    EscStatusSensor escStatusSensor;
    FuelTankSensor fuelTankSensor;
    BatteryInfoSensor batteryInfoSensor;

    geodetic_converter::GeodeticConverter geodeticConverter;
    UavDynamicsSimBase* _uavDynamicsSim;
};

#endif  // SRC_SENSORS_SENSORS_HPP_

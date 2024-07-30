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
#include "battery.hpp"
#include "differential_pressure.hpp"
#include "esc_status.hpp"
#include "fuel_tank.hpp"
#include "gnss.hpp"
#include "ice.hpp"
#include "imu.hpp"
#include "mag.hpp"
#include "velocity.hpp"

#include "uavDynamicsSimBase.hpp"
#include "UavDynamics/math/geodetic.hpp"

struct Sensors {
    explicit Sensors(ros::NodeHandle* nh);
    int8_t init(const std::shared_ptr<UavDynamicsSimBase>& uavDynamicsSim);
    void publishStateToCommunicator(uint8_t dynamicsNotation);

    AttitudeSensor attitudeSensor;
    PressureSensor pressureSensor;
    TemperatureSensor temperatureSensor;
    DiffPressureSensor diffPressureSensor;
    IceStatusSensor iceStatusSensor;
    ImuSensor imuSensor;
    VelocitySensor velocitySensor_;
    GpsSensor gpsSensor;
    MagSensor magSensor;

    EscStatusSensor escStatusSensor;
    FuelTankSensor fuelTankSensor;
    BatteryInfoSensor batteryInfoSensor;

private:
    CoordinateConverter geodeticConverter;
    std::shared_ptr<UavDynamicsSimBase> _uavDynamicsSim;
};

#endif  // SRC_SENSORS_SENSORS_HPP_

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

#include "fuel_tank.hpp"
#include <std_msgs/UInt8.h>

FuelTankSensor::FuelTankSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<std_msgs::UInt8>(topic, 5);
}
bool FuelTankSensor::publish(double fuelLevelPercentage) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(_isEnabled && (nextPubTimeSec_ < crntTimeSec)){
        std_msgs::UInt8 fuelTankMsg;
        fuelTankMsg.data = static_cast<uint8_t>(fuelLevelPercentage);
        publisher_.publish(fuelTankMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD;
    }
    return true;
}

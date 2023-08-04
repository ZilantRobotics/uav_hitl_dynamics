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

#include "battery.hpp"
#include <sensor_msgs/BatteryState.h>

BatteryInfoSensor::BatteryInfoSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<sensor_msgs::BatteryState>(topic, 16);
}
bool BatteryInfoSensor::publish(float percentage) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(_isEnabled && (nextPubTimeSec_ < crntTimeSec)){
        sensor_msgs::BatteryState batteryInfoMsg;
        batteryInfoMsg.voltage = 15.1f;
        batteryInfoMsg.percentage = percentage;
        batteryInfoMsg.capacity = 6;
        publisher_.publish(batteryInfoMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD;
    }
    return true;
}

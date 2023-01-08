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

#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

///< uavcan_msgs are deprecated and should be removed asap
#include <uavcan_msgs/EscStatus.h>
#include <uavcan_msgs/IceFuelTankStatus.h>


EscStatusSensor::EscStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::EscStatus>(topic, 10);
}
bool EscStatusSensor::publish(const std::vector<double>& rpm) {
    ///< The idea here is to publish each esc status with equal interval instead of burst
    auto crntTimeSec = ros::Time::now().toSec();
    if(_isEnabled && rpm.size() > 0 && rpm.size() <= 8 && (nextPubTimeSec_ < crntTimeSec)){
        uavcan_msgs::EscStatus escStatusMsg;
        if(nextEscIdx_ >= rpm.size()){
            nextEscIdx_ = 0;
        }
        escStatusMsg.esc_index = nextEscIdx_;
        escStatusMsg.rpm = rpm[nextEscIdx_];
        publisher_.publish(escStatusMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD / rpm.size();
        nextEscIdx_++;
    }
    return true;
}


FuelTankSensor::FuelTankSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::IceFuelTankStatus>(topic, 16);
}
bool FuelTankSensor::publish(double fuelLevelPercentage) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(_isEnabled && (nextPubTimeSec_ < crntTimeSec)){
        uavcan_msgs::IceFuelTankStatus fuelTankMsg;
        fuelTankMsg.available_fuel_volume_percent = fuelLevelPercentage;
        publisher_.publish(fuelTankMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD;
    }
    return true;
}

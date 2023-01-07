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

#include "attitude.hpp"
#include <geometry_msgs/QuaternionStamped.h>


AttitudeSensor::AttitudeSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<geometry_msgs::QuaternionStamped>(topic, 5);
}
bool AttitudeSensor::publish(const Eigen::Quaterniond& attitudeFrdToNed) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!_isEnabled || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    geometry_msgs::QuaternionStamped msg;
    msg.quaternion.x = attitudeFrdToNed.x();
    msg.quaternion.y = attitudeFrdToNed.y();
    msg.quaternion.z = attitudeFrdToNed.z();
    msg.quaternion.w = attitudeFrdToNed.w();
    msg.header.stamp = ros::Time::now();

    publisher_.publish(msg);
    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

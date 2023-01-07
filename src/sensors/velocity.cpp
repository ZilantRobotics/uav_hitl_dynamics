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

#include "velocity.hpp"
#include <geometry_msgs/Twist.h>

VelocitySensor::VelocitySensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<geometry_msgs::Twist>(topic, 5);
}
bool VelocitySensor::publish(const Eigen::Vector3d& linVelNed, const Eigen::Vector3d& angVelFrd) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!_isEnabled || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = linVelNed[0];
    msg.linear.y = linVelNed[1];
    msg.linear.z = linVelNed[2];
    msg.angular.x = angVelFrd[0];
    msg.angular.y = angVelFrd[1];
    msg.angular.z = angVelFrd[2];

    publisher_.publish(msg);
    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

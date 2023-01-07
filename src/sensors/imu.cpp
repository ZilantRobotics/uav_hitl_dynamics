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

#include "imu.hpp"
#include <sensor_msgs/Imu.h>


ImuSensor::ImuSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<sensor_msgs::Imu>(topic, 5);
}
bool ImuSensor::publish(const Eigen::Vector3d& accFrd, const Eigen::Vector3d& gyroFrd) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!_isEnabled || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.angular_velocity.x = gyroFrd[0];
    msg.angular_velocity.y = gyroFrd[1];
    msg.angular_velocity.z = gyroFrd[2];
    msg.linear_acceleration.x = accFrd[0];
    msg.linear_acceleration.y = accFrd[1];
    msg.linear_acceleration.z = accFrd[2];

    publisher_.publish(msg);
    if (nextPubTimeSec_ + PERIOD > crntTimeSec) {
        nextPubTimeSec_ += PERIOD;
    } else {
        nextPubTimeSec_ = crntTimeSec + PERIOD;
    }
    return true;
}

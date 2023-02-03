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

#include "gnss.hpp"
#include <sensor_msgs/NavSatFix.h>
#include "cs_converter.hpp"

GpsSensor::GpsSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<sensor_msgs::NavSatFix>(topic, 5);
}
bool GpsSensor::publish(const Eigen::Vector3d& gpsPosition) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!_isEnabled || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    sensor_msgs::NavSatFix gps_position_msg;
    gps_position_msg.header.stamp = ros::Time::now();
    gps_position_msg.latitude = gpsPosition[0];
    gps_position_msg.longitude = gpsPosition[1];
    gps_position_msg.altitude = gpsPosition[2];
    publisher_.publish(gps_position_msg);

    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

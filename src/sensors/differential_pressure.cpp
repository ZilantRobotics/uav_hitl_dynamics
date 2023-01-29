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

#include "differential_pressure.hpp"
#include <std_msgs/Float32.h>

static const double STATIC_PRESSURE_NOISE = 0.5;

DiffPressureSensor::DiffPressureSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<std_msgs::Float32>(topic, 5);
}
bool DiffPressureSensor::publish(float diffPressureHpa) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!_isEnabled || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    std_msgs::Float32 msg;
    msg.data = diffPressureHpa * 100;
    msg.data += static_cast<float>(STATIC_PRESSURE_NOISE * normalDistribution_(randomGenerator_));
    publisher_.publish(msg);

    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

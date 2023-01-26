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

#include "esc_status.hpp"
#include <mavros_msgs/ESCTelemetryItem.h>


EscStatusSensor::EscStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<mavros_msgs::ESCTelemetryItem>(topic, 10);
}
bool EscStatusSensor::publish(const std::vector<double>& rpm) {
    ///< The idea here is to publish each esc status with equal interval instead of burst
    auto crntTimeSec = ros::Time::now().toSec();
    if(_isEnabled && !rpm.empty() && rpm.size() <= 8 && (nextPubTimeSec_ < crntTimeSec)){
        mavros_msgs::ESCTelemetryItem escStatusMsg;
        if(nextEscIdx_ >= rpm.size()){
            nextEscIdx_ = 0;
        }
        escStatusMsg.count = nextEscIdx_;
        escStatusMsg.rpm = static_cast<float>(rpm[nextEscIdx_]);
        publisher_.publish(escStatusMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD / (double)rpm.size();
        nextEscIdx_++;
    }
    return true;
}

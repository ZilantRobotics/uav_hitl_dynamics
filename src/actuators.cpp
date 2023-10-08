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

#include "actuators.hpp"

void Actuators::init(ros::NodeHandle& node){
    _actuatorsSub = node.subscribe("/uav/actuators", 1, &Actuators::_actuatorsCallback, this);
    _armSub = node.subscribe("/uav/arm", 1, &Actuators::_armCallback, this);
}

void Actuators::retriveStats(uint64_t* msg_counter, uint64_t* max_delay_us) {
    *msg_counter = _msgCounter;
    *max_delay_us = _maxDelayUsec < 1000000 ? _maxDelayUsec : 0;

    _msgCounter = 0;
    _maxDelayUsec = 0;
}

ArmingStatus Actuators::getArmingStatus() {
    if (ros::Time::now().toSec() > _lastArmingStatusTimestampSec + 2.0) {
        _armingStatus = ArmingStatus::UNKNOWN;
        return ArmingStatus::UNKNOWN;
    }

    return _armingStatus;
}


void Actuators::_actuatorsCallback(sensor_msgs::Joy::Ptr msg){
    uint64_t crntTimeUs = ros::Time::now().toSec() * 1000000;

    auto crntDelayUsec = crntTimeUs - _lastActuatorsTimestampUsec;
    if(crntDelayUsec > _maxDelayUsec){
        _maxDelayUsec = crntDelayUsec;
    }
    _lastActuatorsTimestampUsec = crntTimeUs;
    _msgCounter++;

    actuatorsSize = std::min(msg->axes.size(), actuators.size());
    for(size_t idx = 0; idx < actuatorsSize; idx++){
        actuators[idx] = msg->axes[idx];
    }

    if (_scenarioType == 1) {
        actuators[7] = 0.0;
    }
}

void Actuators::_armCallback(std_msgs::Bool msg){
    auto new_arming_status = msg.data ? ArmingStatus::ARMED : ArmingStatus::DISARMED;

    if(new_arming_status != _armingStatus){
        /**
         * @note why it publish few times when sim starts? hack: use throttle
         */
        ROS_INFO_STREAM_THROTTLE(1, "ArmingStatus: " << (msg.data ? "Arm" : "Disarm"));
        _armingStatus = msg.data ? ArmingStatus::ARMED : ArmingStatus::DISARMED;
    }

    _lastArmingStatusTimestampSec = ros::Time::now().toSec();
}

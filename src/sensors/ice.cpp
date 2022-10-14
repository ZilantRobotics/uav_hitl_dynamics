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

#include "ice.hpp"

static const constexpr double WORKING_RPM = 4000.0;
static const constexpr double STARTING_RPM = 500.0;
static const constexpr double PERIOD_1 = 1500.0;    // falling rpm period
static const constexpr double PERIOD_2 = 1500.0;    // starting period
static const constexpr double PERIOD_3 = 2000.0;    // waiting period
static const constexpr double PERIOD_23 = PERIOD_2 + PERIOD_3;    // starting + waiting period

IceStatusSensor::IceStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::IceReciprocatingStatus>(topic, 16);
}
bool IceStatusSensor::publish(double rpm) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(_isEnabled && (nextPubTimeSec_ < crntTimeSec)){
        estimate_state(rpm);
        publisher_.publish(_iceStatusMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD;
    }
    return true;
}
void IceStatusSensor::estimate_state(double rpm) {
    if (_stallTsMs == 0) {
        emulate_normal_mode(rpm);
    } else {
        emulate_stall_mode();
    }
}

void IceStatusSensor::emulate_normal_mode(double rpm) {
    auto crntTimeSec = ros::Time::now().toSec();
    if (rpm < 1.0) {
        _iceStatusMsg.state = 0;
    } else if (_iceStatusMsg.state == 0) {
        _iceStatusMsg.state = 1;
        _startTsSec = ros::Time::now().toSec();
    } else if (_startTsSec + 3.0 < crntTimeSec) {
        _iceStatusMsg.state = 2;
    }
    _iceStatusMsg.engine_speed_rpm = rpm;
}

void IceStatusSensor::emulate_stall_mode() {
    auto crntTimeMs = ros::Time::now().toSec() * 1000;
    auto timeElapsedMs = crntTimeMs - _stallTsMs;
    if (timeElapsedMs < PERIOD_1) {
        _iceStatusMsg.state = 2;
        _iceStatusMsg.engine_speed_rpm = WORKING_RPM * (PERIOD_1 - timeElapsedMs) / PERIOD_1;
    } else {
        double timeSinceRestartMs = timeElapsedMs - PERIOD_1;
        if (fmod(timeSinceRestartMs, PERIOD_23) < PERIOD_2) {
            _iceStatusMsg.state = 1;
            _iceStatusMsg.engine_speed_rpm = STARTING_RPM;
        } else {
            _iceStatusMsg.state = 2;
            _iceStatusMsg.engine_speed_rpm = 0.0;
        }
    }
}

void IceStatusSensor::start_stall_emulation() {
    _stallTsMs = ros::Time::now().toSec() * 1000;
}

void IceStatusSensor::stop_stall_emulation() {
    _stallTsMs = 0;
}

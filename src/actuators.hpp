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

#ifndef SRC_ACTUATORS_HPP
#define SRC_ACTUATORS_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>


enum class ArmingStatus {
    UNKNOWN,    // Vehicle without Arming status is assumed to be always armed
    ARMED,
    DISARMED,
};

struct Actuators {
    Actuators() : actuators(16, 0.0) {}
    void init(ros::NodeHandle& node);
    void retriveStats(uint64_t* msg_counter, uint64_t* max_delay_us);
    ArmingStatus getArmingStatus();

    std::vector<double> actuators;
    uint8_t actuatorsSize{0};
    uint8_t _scenarioType{0};

private:
    void _actuatorsCallback(sensor_msgs::Joy::Ptr msg);
    void _armCallback(std_msgs::Bool msg);

    ros::Subscriber _actuatorsSub;
    ros::Subscriber _armSub;
    uint64_t _lastActuatorsTimestampUsec;

    uint64_t _maxDelayUsec{0};
    uint64_t _msgCounter{0};

    ArmingStatus _armingStatus{ArmingStatus::DISARMED};
    double _lastArmingStatusTimestampSec{ros::Time::now().toSec()};
};

#endif  // SRC_ACTUATORS_HPP

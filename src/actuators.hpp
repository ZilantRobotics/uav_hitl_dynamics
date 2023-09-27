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

struct Actuators {
    Actuators() : _actuators(8, 0.0) {}
    void init(ros::NodeHandle& node){
        actuatorsSub_ = node.subscribe("/uav/actuators", 1, &Actuators::actuatorsCallback, this);
        armSub_ = node.subscribe("/uav/arm", 1, &Actuators::armCallback, this);
    }

    ros::Subscriber actuatorsSub_;
    std::vector<double> _actuators;
    uint64_t lastActuatorsTimestampUsec_;
    uint64_t prevActuatorsTimestampUsec_;
    uint64_t maxDelayUsec_;
    uint64_t actuatorsMsgCounter_{0};
    void actuatorsCallback(sensor_msgs::Joy::Ptr msg);

    uint8_t _scenarioType{0};

    ros::Subscriber armSub_;
    bool armed_ = true;  // temp hack for vehicle without known ArmingStatus
    void armCallback(std_msgs::Bool msg);
};

#endif  // SRC_ACTUATORS_HPP

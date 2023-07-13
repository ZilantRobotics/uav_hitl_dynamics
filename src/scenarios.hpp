/*
 * Copyright (c) 2023 RaccoonLab.
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

#ifndef UAV_DYNAMICS_SCENARIOS_HPP
#define UAV_DYNAMICS_SCENARIOS_HPP

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <actuators.hpp>
#include <sensors.hpp>

enum class Scenario {
    BARO_DISABLE = 0,
    BARO_ENABLE,

    DIFFERENTIAL_PRESSURE_DISABLE,
    DIFFERENTIAL_PRESSURE_ENABLE,

    GNSS_DISABLE,
    GNSS_ENABLE,

    ICE_START_STALL_EMULATION,
    ICE_STOP_STALL_EMULATION,

    MAG_DISABLE,
    MAG_ENABLE,

    ESC_FEEDBACK_DISABLE,
    ESC_FEEDBACK_ENABLE,
};

struct ScenarioManager {
    ScenarioManager(ros::NodeHandle& node, Actuators& actuators, Sensors& sensors) :
        _node(node), _actuators(actuators), _sensors(sensors) {}
    void init();
private:
    ros::Subscriber _scenarioSub;
    ros::NodeHandle& _node;
    Actuators& _actuators;
    Sensors& _sensors;

    void scenarioCallback(std_msgs::UInt8 msg);
};

#endif  // UAV_DYNAMICS_SCENARIOS_HPP

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

#include "logger.hpp"

#include <Eigen/Geometry>

#include "main.hpp"
#include "cs_converter.hpp"

static const std::string COLOR_RED = "\033[1;31m";
static const std::string COLOR_GREEN = "\033[1;32m";
static const std::string COLOR_BOLD = "\033[1;29m";
static const std::string COLOR_TAIL = "\033[0m";

static const float ROS_PUB_PERIOD_SEC = 0.05f;

void StateLogger::init(double clockScale, double dt_secs) {
    _clockScale = clockScale;
    _dt_secs = dt_secs;
}

void StateLogger::createStringStream(std::stringstream& logStream,
                                     const Eigen::Vector3d& pose,
                                     double dynamicsCounter,
                                     double rosPubCounter,
                                     double periodSec) {

    auto& actuators = _actuators._actuators;
    auto& maxDelayUsec = _actuators.maxDelayUsec_;
    auto& actuatorsMsgCounter = _actuators.actuatorsMsgCounter_;
    const auto& armed = _actuators.armed_;

    std::string arm_str = armed ? COLOR_GREEN + "[Armed]" + COLOR_TAIL : "[Disarmed]";
    logStream << arm_str << ", ";

    logStream << _info.dynamicsName.c_str() << ". ";

    double dynamicsCompleteness = (double)dynamicsCounter * _dt_secs / (_clockScale * periodSec);
    std::string dyn_str = "dyn=" + std::to_string(dynamicsCompleteness);
    colorize(logStream, dynamicsCompleteness >= 0.9, dyn_str);
    logStream << ", ";

    double rosPubCompleteness = (double)rosPubCounter * (double)ROS_PUB_PERIOD_SEC / (_clockScale * periodSec);
    std::string ros_pub_str = "ros_pub=" + std::to_string(rosPubCompleteness);
    colorize(logStream, rosPubCompleteness >= 0.9, ros_pub_str);
    logStream << ", ";

    std::string actuator_str = "setpoint=" + std::to_string(actuatorsMsgCounter);
    bool is_actuator_ok = actuatorsMsgCounter > 100 && maxDelayUsec < 20000 && maxDelayUsec != 0;
    colorize(logStream, is_actuator_ok, actuator_str);
    logStream << " msg/sec.\n";
    actuatorsMsgCounter = 0;
    maxDelayUsec = 0;

    addBold(logStream, "mc");
    logStream << std::setprecision(2) << std::fixed << " ["
                << actuators[0] << ", "
                << actuators[1] << ", "
                << actuators[2] << ", "
                << actuators[3] << "] ";

    if(_info.vehicleType == VehicleType::INNOPOLIS_VTOL){
        addBold(logStream, "fw rpy");
        logStream << " [" << actuators[4] << ", "
                            << actuators[5] << ", "
                            << actuators[6] << "]";
        addBold(logStream, " throttle");
        logStream << " [" << actuators[7] << "] ";
    }

    auto enuPosition = (_info.notation == DynamicsNotation_t::PX4_NED_FRD) ? Converter::nedToEnu(pose) : pose;
    addBold(logStream, "enu pose");
    logStream << std::setprecision(1) << std::fixed << " ["
                << enuPosition[0] << ", "
                << enuPosition[1] << ", "
                << enuPosition[2] << "].";
}

void StateLogger::colorize(std::stringstream& logStream, bool is_ok, const std::string& newData) {
    if(!is_ok){
        logStream << COLOR_RED << newData << COLOR_TAIL;
    }else{
        logStream << newData;
    }
}

void StateLogger::addBold(std::stringstream& logStream, const char* newData) {
    logStream << COLOR_BOLD << newData << COLOR_TAIL;
}
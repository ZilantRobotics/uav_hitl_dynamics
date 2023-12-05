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
static const std::string COLOR_YELLOW = "\033[1;33m";
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
    uint64_t actuatorsMsgCounter;
    uint64_t actuatorsMaxDelayUsec;
    _actuators.retriveStats(&actuatorsMsgCounter, &actuatorsMaxDelayUsec);
    const auto arming_status = _actuators.getArmingStatus();
    const auto& actuators = _actuators.actuators;
    auto actuatorsSize = _actuators.actuatorsSize;

    std::string arm_str;
    if (arming_status == ArmingStatus::ARMED) {
        arm_str = COLOR_GREEN + "[Armed]" + COLOR_TAIL;
    } else if (arming_status == ArmingStatus::DISARMED) {
        arm_str = "[Disarmed]";
    } else {
        arm_str = COLOR_YELLOW + "[No ARM status]" + COLOR_TAIL;
    }
    logStream << arm_str << ", ";

    logStream << _info.dynamicsName.c_str() << ". ";

    double dynamicsCompleteness = (double)dynamicsCounter * _dt_secs / (_clockScale * periodSec);
    uint8_t dynamicsPercent = 100 * dynamicsCompleteness;
    std::string dyn_str = "dyn=" + std::to_string(dynamicsPercent) + "%";
    addErrColor(logStream, dynamicsPercent >= 95, dyn_str);
    logStream << ", ";

    double rosPubCompleteness = (double)rosPubCounter * (double)ROS_PUB_PERIOD_SEC / (_clockScale * periodSec);
    uint8_t rosPubPercent = 100 * rosPubCompleteness;
    std::string ros_pub_str = "ros_pub=" + std::to_string(rosPubPercent) + "%";
    addErrColor(logStream, rosPubPercent >= 99, ros_pub_str);
    logStream << ", ";

    std::string setpoint_name;
    if (actuatorsSize > 1) {
        setpoint_name = "Vector" + std::to_string(actuatorsSize);
    } else if (actuatorsSize == 1) {
        setpoint_name = "Scalar";
    } else {
        setpoint_name = "Setpoint";
    }
    std::string actuator_str = setpoint_name + "=" + std::to_string(actuatorsMsgCounter) +\
                               " hz (" + std::to_string(actuatorsMaxDelayUsec / 1000) + " ms)";
    if (actuatorsMsgCounter < 150 || actuatorsMaxDelayUsec > 30000 || actuatorsMaxDelayUsec == 0) {
        addErrColor(logStream, false, actuator_str);
    } else if (actuatorsMsgCounter < 175 || actuatorsMaxDelayUsec > 15000) {
        addWarnColor(logStream, actuator_str);
    } else {
        logStream << actuator_str;
    }
    logStream << ".\n";

    addBold(logStream, "mc");
    logStream << std::setprecision(2) << std::fixed << " ["
                << actuators[0] << ", "
                << actuators[1] << ", "
                << actuators[2] << ", "
                << actuators[3] << "] ";

    if(_info.loggingType == LoggingType::STANDARD_VTOL){
        addBold(logStream, "fw rpy");
        logStream << " [" << actuators[5] << ", "
                            << actuators[6] << ", "
                            << actuators[7] << "]";
        addBold(logStream, " throttle");
        logStream << " [" << actuators[4] << "] ";
    }

    auto enuPosition = (_info.notation == DynamicsNotation_t::PX4_NED_FRD) ? Converter::nedToEnu(pose) : pose;
    addBold(logStream, "enu pose");
    logStream << std::setprecision(1) << std::fixed << " ["
                << enuPosition[0] << ", "
                << enuPosition[1] << ", "
                << enuPosition[2] << "].";
}

void StateLogger::addErrColor(std::stringstream& logStream, bool is_ok, const std::string& newData) {
    if(!is_ok){
        logStream << COLOR_RED << newData << COLOR_TAIL;
    }else{
        logStream << newData;
    }
}

void StateLogger::addWarnColor(std::stringstream& logStream, const std::string& newData) {
    logStream << COLOR_YELLOW << newData << COLOR_TAIL;
}

void StateLogger::addBold(std::stringstream& logStream, const char* newData) {
    logStream << COLOR_BOLD << newData << COLOR_TAIL;
}

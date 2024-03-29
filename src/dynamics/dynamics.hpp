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

#ifndef SRC_DYNAMICS_DYNAMICS_HPP
#define SRC_DYNAMICS_DYNAMICS_HPP

enum class DynamicsType{
    QUADCOPTER = 0,
    VTOL,
    OCTOCOPTER,
};

enum class LoggingType{
    QUADCOPTER = 0,
    STANDARD_VTOL,
    VTOL_8_MOTORS,
};

enum class DynamicsNotation_t{
    PX4_NED_FRD = 0,
    ROS_ENU_FLU = 1,
};

struct DynamicsInfo {
    std::string dynamicsName;
    DynamicsType dynamicsType;

    LoggingType loggingType;
    std::string loggingTypeName;

    DynamicsNotation_t notation;
};


#endif  // SRC_DYNAMICS_DYNAMICS_HPP

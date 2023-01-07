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

#ifndef SRC_SENSORS_ATTITUDE_HPP
#define SRC_SENSORS_ATTITUDE_HPP

#include "sensor_base.hpp"
#include <Eigen/Geometry>

class AttitudeSensor : public BaseSensor{
    public:
        AttitudeSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(const Eigen::Quaterniond& attitudeFrdToNed);
};

#endif  // SRC_SENSORS_ATTITUDE_HPP

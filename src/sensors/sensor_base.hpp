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

#ifndef SENSORS_SENSOR_BASE_HPP
#define SENSORS_SENSOR_BASE_HPP

#include <ros/ros.h>
// #include <ros/time.h>
#include <random>
#include <geographiclib_conversions/geodetic_conv.hpp>

class BaseSensor{
    public:
        BaseSensor() = delete;
        BaseSensor(ros::NodeHandle* nh, double period): node_handler_(nh), PERIOD(period) {};
        void enable() {_isEnabled = true;}
        void disable() {_isEnabled = false;}
    protected:
        ros::NodeHandle* node_handler_;
        bool _isEnabled{false};
        const double PERIOD;
        ros::Publisher publisher_;
        double nextPubTimeSec_ = 0;

        std::default_random_engine randomGenerator_;
        std::normal_distribution<double> normalDistribution_{std::normal_distribution<double>(0.0, 1.0)};
};

#endif  // SENSORS_SENSOR_BASE_HPP

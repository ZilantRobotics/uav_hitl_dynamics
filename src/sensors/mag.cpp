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

#include "mag.hpp"
#include <sensor_msgs/MagneticField.h>
#include "cs_converter.hpp"
#include "UavDynamics/math/wmm.hpp"

static const double MAG_NOISE = 0.0002;


MagSensor::MagSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<sensor_msgs::MagneticField>(topic, 5);
}
bool MagSensor::publish(const Eigen::Vector3d& geoPosition, const Eigen::Quaterniond& attitudeFrdToNed) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!_isEnabled || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    sensor_msgs::MagneticField msg;
    Eigen::Vector3d magEnu;
    calculateMagneticFieldStrengthGauss(
        geoPosition.x(), geoPosition.y(), geoPosition.z(),
        magEnu.x(), magEnu.y(), magEnu.z()
    );

    magEnu.z() = -1 * magEnu.z();

    Eigen::Vector3d magFrd = attitudeFrdToNed.inverse() * Converter::enuToNed(magEnu);
    msg.header.stamp = ros::Time();
    msg.magnetic_field.x = magFrd[0] + MAG_NOISE * normalDistribution_(randomGenerator_);
    msg.magnetic_field.y = magFrd[1] + MAG_NOISE * normalDistribution_(randomGenerator_);
    msg.magnetic_field.z = magFrd[2] + MAG_NOISE * normalDistribution_(randomGenerator_);

    publisher_.publish(msg);
    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

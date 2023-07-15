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


#include "quadcopter.hpp"
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

void QuadcopterDynamics::initStaticMotorTransform(double momentArm){
    Eigen::Isometry3d motorFrame = Eigen::Isometry3d::Identity();

    motorFrame.translation() = Eigen::Vector3d(momentArm, momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, 1, 0);

    motorFrame.translation() = Eigen::Vector3d(-momentArm, momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, -1, 1);

    motorFrame.translation() = Eigen::Vector3d(-momentArm, -momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, 1, 2);

    motorFrame.translation() = Eigen::Vector3d(momentArm, -momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, -1, 3);
}

std::vector<double> QuadcopterDynamics::mapCmdActuator(std::vector<double> initialCmd) const{
    std::vector<double> mappedCmd;
    mappedCmd.push_back(initialCmd[2]);     // PX4: motor 3, front left
    mappedCmd.push_back(initialCmd[1]);     // PX4: motor 2, tail left
    mappedCmd.push_back(initialCmd[3]);     // PX4: motor 4, tail right
    mappedCmd.push_back(initialCmd[0]);     // PX4: motor 1, front right
    return mappedCmd;
}

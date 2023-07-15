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


#include "octocopter.hpp"
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

void OctocopterDynamics::initStaticMotorTransform(double momentArm){
    Eigen::Isometry3d motorFrame = Eigen::Isometry3d::Identity();

    motorFrame.translation() = Eigen::Vector3d(momentArm, momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, 1, 0);

    motorFrame.translation() = Eigen::Vector3d(-momentArm, momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, -1, 1);

    motorFrame.translation() = Eigen::Vector3d(-momentArm, -momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, 1, 2);

    motorFrame.translation() = Eigen::Vector3d(momentArm, -momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, -1, 3);

    motorFrame.translation() = Eigen::Vector3d(momentArm, momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, 1, 4);

    motorFrame.translation() = Eigen::Vector3d(-momentArm, momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, -1, 5);

    motorFrame.translation() = Eigen::Vector3d(-momentArm, -momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, 1, 6);

    motorFrame.translation() = Eigen::Vector3d(momentArm, -momentArm, 0.);
    multicopterSim_->setMotorFrame(motorFrame, -1, 7);
}

std::vector<double> OctocopterDynamics::mapCmdActuator(std::vector<double> initialCmd) const{
    std::vector<double> mappedCmd;
    mappedCmd.push_back(initialCmd[1]);
    mappedCmd.push_back(initialCmd[2]);
    mappedCmd.push_back(initialCmd[3]);
    mappedCmd.push_back(initialCmd[0]);

    mappedCmd.push_back(initialCmd[4]);
    mappedCmd.push_back(initialCmd[7]);
    mappedCmd.push_back(initialCmd[6]);
    mappedCmd.push_back(initialCmd[5]);

    return mappedCmd;
}

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

#ifndef SRC_DYNAMICS_QUADCOPTER_QUADCOPTER_HPP
#define SRC_DYNAMICS_QUADCOPTER_QUADCOPTER_HPP

#include "uavDynamicsSimBase.hpp"
#include "multirotor.hpp"

class QuadcopterDynamics: public MultirotorDynamics{
public:
    QuadcopterDynamics() : MultirotorDynamics() {
        number_of_motors = 4;
    }
    ~QuadcopterDynamics() final = default;

    void initStaticMotorTransform(double momentArm) override;
    std::vector<double> mapCmdActuator(std::vector<double> cmd) const override;
};

#endif  // SRC_DYNAMICS_QUADCOPTER_QUADCOPTER_HPP

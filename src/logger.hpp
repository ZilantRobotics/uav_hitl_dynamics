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

#ifndef UAV_DYNAMICS_LOGER_HPP
#define UAV_DYNAMICS_LOGER_HPP

#include "actuators.hpp"
#include "sensors.hpp"
#include "dynamics.hpp"

struct StateLogger {
    StateLogger(Actuators& actuators, Sensors& sensors, DynamicsInfo& info) :
         _actuators(actuators), _sensors(sensors), _info(info) {}
    void init(double clockScale, double dt_secs);
    void createStringStream(std::stringstream& logStream,
                            const Eigen::Vector3d& pose,
                            double dynamicsCounter,
                            double rosPubCounter,
                            double periodSec);

private:
    static void addErrColor(std::stringstream& logStream, bool is_ok, const std::string& newData);
    static void addWarnColor(std::stringstream& logStream, const std::string& newData);
    static void addBold(std::stringstream& logStream, const char* newData);

    Actuators& _actuators;
    Sensors& _sensors;
    DynamicsInfo& _info;

    double _clockScale;
    double _dt_secs;
};

#endif  // UAV_DYNAMICS_LOGER_HPP

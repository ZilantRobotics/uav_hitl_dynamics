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

#include "scenarios.hpp"

void ScenarioManager::init() {
    _scenarioSub = _node.subscribe("/uav/scenario", 1, &ScenarioManager::scenarioCallback, this);
}

void ScenarioManager::scenarioCallback(std_msgs::UInt8 msg){
    Scenario scenarioType = static_cast<Scenario>(msg.data);

    switch(scenarioType) {
        case Scenario::BARO_DISABLE:
            _sensors.pressureSensor.disable();
            break;
        case Scenario::BARO_ENABLE:
            _sensors.pressureSensor.enable();
            break;

        case Scenario::DIFFERENTIAL_PRESSURE_DISABLE:
            _sensors.diffPressureSensor.disable();
            break;
        case Scenario::DIFFERENTIAL_PRESSURE_ENABLE:
            _sensors.diffPressureSensor.enable();
            break;

        case Scenario::ICE_STOP_STALL_EMULATION:
            _actuators._scenarioType = msg.data;
            _sensors.iceStatusSensor.stop_stall_emulation();
            break;
        case Scenario::ICE_START_STALL_EMULATION:
            _actuators._scenarioType = msg.data;
            _sensors.iceStatusSensor.start_stall_emulation();
            break;

        case Scenario::GNSS_DISABLE:
            _sensors.gpsSensor.disable();
            break;
        case Scenario::GNSS_ENABLE:
            _sensors.gpsSensor.enable();
            break;

        case Scenario::MAG_DISABLE:
            _sensors.magSensor.disable();
            break;
        case Scenario::MAG_ENABLE:
            _sensors.magSensor.enable();
            break;

        case Scenario::ESC_FEEDBACK_DISABLE:
            _sensors.escStatusSensor.disable();
            break;
        case Scenario::ESC_FEEDBACK_ENABLE:
            _sensors.escStatusSensor.enable();
            break;

        default:
            break;
    }
}

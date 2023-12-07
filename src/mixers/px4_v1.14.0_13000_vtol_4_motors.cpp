/*
 * Copyright (c) 2023 Dmitry Ponomarev <ponomarevda96@gmail.com>
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

#include "px4_v1.14.0_13000_vtol_4_motors.hpp"

int8_t PX4_V_1_14_0_Airframe_13000_4_motors::init() {
    actuatorsPub = _node.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC, 5);

    motorsSub = _node.subscribe(MOTORS_TOPIC, 2, &PX4_V_1_14_0_Airframe_13000_4_motors::motorsCallback, this);
    servosSub = _node.subscribe(SERVOS_TOPIC, 2, &PX4_V_1_14_0_Airframe_13000_4_motors::servosCallback, this);

    for (uint_fast8_t idx = 0; idx < 8; idx++) {
        actuatorsMsg.axes.push_back(0);
    }

    return 0;
}
void PX4_V_1_14_0_Airframe_13000_4_motors::motorsCallback(sensor_msgs::Joy msg) {
    auto& axes = actuatorsMsg.axes;
    if (msg.axes.size() >= 4) {
        axes[VTOL_MOTOR_0_FRONT_RIGHT] = clamp_float(msg.axes[0], 0.0, 1.0);
        axes[VTOL_MOTOR_1_REAR_LEFT] = clamp_float(msg.axes[1], 0.0, 1.0);
        axes[VTOL_MOTOR_2_FRONT_LEFT] = clamp_float(msg.axes[2], 0.0, 1.0);
        axes[VTOL_MOTOR_3_REAR_RIGHT] = clamp_float(msg.axes[3], 0.0, 1.0);
    }
    if (msg.axes.size() >= 5) {
        axes[VTOL_THROTLE] = clamp_float(msg.axes[INPUT_THROTLE], 0.0, 1.0);
    }
    if (msg.axes.size() >= 9) {
        if (abs(msg.axes[INPUT_AILERON_2]) < 0.001 &&
                abs(msg.axes[INPUT_ELEVATORS]) < 0.001 &&
                abs(msg.axes[INPUT_RUDDERS]) < 0.001) {
            axes[VTOL_AILERONS] = 0.0;
            axes[VTOL_ELEVATORS] = 0.0;
            axes[VTOL_RUDDERS] = 0.0;
        } else {
            axes[VTOL_AILERONS] = 2.0 * clamp_float(msg.axes[INPUT_AILERON_2], 0.0, 1.0) - 1.0;
            axes[VTOL_ELEVATORS] = 2.0 * clamp_float(msg.axes[INPUT_ELEVATORS], 0.0, 1.0) - 1.0;
            axes[VTOL_RUDDERS] = 2.0 * clamp_float(msg.axes[INPUT_RUDDERS], 0.0, 1.0) - 1.0;
        }
    }

    actuatorsPub.publish(actuatorsMsg);
}
void PX4_V_1_14_0_Airframe_13000_4_motors::servosCallback(sensor_msgs::Joy msg) {
    ///< ignore left aileron msg.axes[0] here
    actuatorsMsg.axes[VTOL_AILERONS] = clamp_float(msg.axes[1], -1.0, 1.0);
    actuatorsMsg.axes[VTOL_ELEVATORS] = clamp_float(msg.axes[2], -1.0, 1.0);
    actuatorsMsg.axes[VTOL_RUDDERS] = clamp_float(msg.axes[3], -1.0, 1.0);
}

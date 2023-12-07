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

#include "px4_v1.14.0_13000_vtol_8_motors.hpp"

static const constexpr size_t MOTORS_AMOUNT = 9;
static const constexpr size_t ACTUATORS_AMOUNT = 13;

int8_t PX4_V_1_14_0_Airframe_13000_8_motors::init() {
    actuatorsPub = _node.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC, 5);

    motorsSub = _node.subscribe(MOTORS_TOPIC, 2, &PX4_V_1_14_0_Airframe_13000_8_motors::motorsCallback, this);
    servosSub = _node.subscribe(SERVOS_TOPIC, 2, &PX4_V_1_14_0_Airframe_13000_8_motors::servosCallback, this);

    actuatorsMsg.axes.resize(ACTUATORS_AMOUNT, 0.0);

    return 0;
}
void PX4_V_1_14_0_Airframe_13000_8_motors::motorsCallback(sensor_msgs::Joy msg) {
    auto& axes = actuatorsMsg.axes;

    // motors
    if (msg.axes.size() >= MOTORS_AMOUNT) {
        for (size_t idx = 0; idx < MOTORS_AMOUNT; idx++) {
            axes[idx] = clamp_float(msg.axes[idx], 0.0, 1.0);
        }
    }

    // servos
    if (msg.axes.size() >= ACTUATORS_AMOUNT) {
        axes[VTOL_OP_AILERONS] = rawcommand_to_servo(msg.axes[INPUT_AILERON_2]);
        axes[VTOL_OP_ELEVATORS] = rawcommand_to_servo(msg.axes[INPUT_ELEVATORS]);
        axes[VTOL_OP_RUDDERS] = rawcommand_to_servo(msg.axes[INPUT_RUDDERS]);
    }

    actuatorsPub.publish(actuatorsMsg);
}
void PX4_V_1_14_0_Airframe_13000_8_motors::servosCallback(sensor_msgs::Joy msg) {
    ///< ignore left aileron msg.axes[0] here
    actuatorsMsg.axes[VTOL_OP_AILERONS] = clamp_float(msg.axes[1], -1.0, 1.0);
    actuatorsMsg.axes[VTOL_OP_ELEVATORS] = clamp_float(msg.axes[2], -1.0, 1.0);
    actuatorsMsg.axes[VTOL_OP_RUDDERS] = clamp_float(msg.axes[3], -1.0, 1.0);
}

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

#include "px4_v1.12.1_13070.hpp"

int8_t PX4_V_1_12_1_Airframe_13070_to_VTOL::init() {
    actuatorsPub = _node.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC, 5);
    motorsSub = _node.subscribe(MOTORS_TOPIC, 2, &PX4_V_1_12_1_Airframe_13070_to_VTOL::motorsCallback, this);

    sp_to_dynamics.axes.resize(8, 0.0f);

    return 0;
}
void PX4_V_1_12_1_Airframe_13070_to_VTOL::motorsCallback(sensor_msgs::Joy sp_from_px4) {
    if (!(sp_from_px4.axes.size() == 4 || sp_from_px4.axes.size() == 8)) {
        return;
    }

    auto& out = sp_to_dynamics.axes;

    if (sp_from_px4.axes.size() == 8) {
        out[VTOL_AILERONS] = clamp_float(sp_from_px4.axes[INPUT_AILERONS], 0.0f, 1.0f) * 2.0f - 1.0f;
        out[VTOL_ELEVATORS] = 1.0f - clamp_float(sp_from_px4.axes[INPUT_ELEVATORS], 0.0f, 1.0f) * 2.0f;
        out[VTOL_RUDDERS] = clamp_float(sp_from_px4.axes[INPUT_RUDDERS], 0.0f, 1.0f) * 2.0f - 1.0f;
        out[VTOL_THROTLE] = sp_from_px4.axes[INPUT_THROTLE] / 0.75f;
    } else if (sp_from_px4.axes.size() == 4) {
        out[VTOL_AILERONS] = 0.0f;
        out[VTOL_ELEVATORS] = 0.0f;
        out[VTOL_RUDDERS] = 0.0f;
        out[VTOL_THROTLE] = 0.0f;
    }

    actuatorsPub.publish(sp_from_px4);
}

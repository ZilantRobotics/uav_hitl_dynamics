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

#include "mixer_babyshark.hpp"

enum BABY_SHARK_OUTPUTS {
    BABY_SHARK_AILERONS = 0,
    BABY_SHARK_A_TAIL_LEFT,
    BABY_SHARK_PUSHER_MOTOR,
    BABY_SHARK_A_TAIL_RIGHT,
    BABY_SHARK_MOTOR_0,
    BABY_SHARK_MOTOR_1,
    BABY_SHARK_MOTOR_2,
    BABY_SHARK_MOTOR_3,
};

enum ACTUATORS_OUTPUT {
    MC_MOTOR_0 = 0,
    MC_MOTOR_1,
    MC_MOTOR_2,
    MC_MOTOR_3,
    VTOL_ROLL,
    VTOL_PITCH,
    VTOL_YAW,
    VTOL_THROTTLE,
};

BabysharkReverseMixer::BabysharkReverseMixer(const ros::NodeHandle& nh) : BaseReverseMixer(nh) {
    for (size_t channel = 0; channel < 8; channel++) {
        sp_to_dynamics.axes.push_back(0);
    }
}

void BabysharkReverseMixer::motorsCallback(sensor_msgs::Joy msg) {
    if (msg.axes.size() == 8) {
        sp_to_dynamics.header = msg.header;

        sp_to_dynamics.axes[MC_MOTOR_0] = msg.axes[BABY_SHARK_MOTOR_0];
        sp_to_dynamics.axes[MC_MOTOR_1] = msg.axes[BABY_SHARK_MOTOR_1];
        sp_to_dynamics.axes[MC_MOTOR_2] = msg.axes[BABY_SHARK_MOTOR_2];
        sp_to_dynamics.axes[MC_MOTOR_3] = msg.axes[BABY_SHARK_MOTOR_3];

        float roll = msg.axes[BABY_SHARK_AILERONS];
        roll = (roll < 0) ? 0.5 : 1 - roll;
        sp_to_dynamics.axes[VTOL_ROLL] = roll;

        float pitch = -msg.axes[BABY_SHARK_A_TAIL_LEFT] + msg.axes[BABY_SHARK_A_TAIL_RIGHT];
        pitch = (pitch < 0) ? 0.0f : pitch / 0.8f;
        sp_to_dynamics.axes[VTOL_PITCH] = pitch;

        float yaw = msg.axes[BABY_SHARK_A_TAIL_LEFT] + msg.axes[BABY_SHARK_A_TAIL_RIGHT];
        yaw = (yaw < 0) ? 0.0f : (1.0f - yaw) / 0.7f;
        sp_to_dynamics.axes[VTOL_YAW] = yaw;

        sp_to_dynamics.axes[VTOL_THROTTLE] = msg.axes[BABY_SHARK_PUSHER_MOTOR];
        actuatorsPub.publish(sp_to_dynamics);
    }
}

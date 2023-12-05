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

#include "base_mixer.hpp"

int8_t BaseReverseMixer::init() {
    actuatorsPub = _node.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC, 5);
    motorsSub = _node.subscribe(MOTORS_TOPIC, 2, &BaseReverseMixer::motorsCallback, this);
    return 0;
}

float clamp_float(float value, float min, float max) {
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    }
    return value;
}

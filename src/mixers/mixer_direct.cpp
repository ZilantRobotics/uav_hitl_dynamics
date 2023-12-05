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

#include "mixer_direct.hpp"

void DirectMixer::motorsCallback(sensor_msgs::Joy msg) {
    if (msg.axes.size() < 4) {
        return;
    }

    auto channels_amount = std::min(MAX_CHANNELS, (uint8_t)msg.axes.size());
    for (uint8_t idx = 0; idx < channels_amount; idx++) {
        sp_to_dynamics.axes[idx] = msg.axes[idx];
    }

    sp_to_dynamics.header = msg.header;
    actuatorsPub.publish(sp_to_dynamics);
}

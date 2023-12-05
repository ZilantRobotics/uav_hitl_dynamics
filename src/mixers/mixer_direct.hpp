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

#ifndef SRC_MIXERS_MIXER_DIRECT_HPP
#define SRC_MIXERS_MIXER_DIRECT_HPP

#include "base_mixer.hpp"

class DirectMixer : public BaseReverseMixer {
    public:
        explicit DirectMixer(const ros::NodeHandle& nh) : BaseReverseMixer(nh) {
            for (size_t channel = 0; channel < MAX_CHANNELS; channel++) {
                sp_to_dynamics.axes.push_back(0);
            }
        }
        ~DirectMixer() final = default;
    protected:
        void motorsCallback(sensor_msgs::Joy msg) override;
        inline static constexpr uint8_t MAX_CHANNELS = 8;
};

#endif  // SRC_MIXERS_MIXER_DIRECT_HPP

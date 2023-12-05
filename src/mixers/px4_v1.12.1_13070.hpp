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

#ifndef SRC_MIXERS_MIXER_PX4_V_1_12_1_13070_HPP
#define SRC_MIXERS_MIXER_PX4_V_1_12_1_13070_HPP

#include "base_mixer.hpp"

class PX4_V_1_12_1_Airframe_13070_to_VTOL : public BaseReverseMixer {
    public:
        using BaseReverseMixer::BaseReverseMixer;
        ~PX4_V_1_12_1_Airframe_13070_to_VTOL() final = default;
        int8_t init() override;
    protected:
        enum InputAirframe {
            INPUT_AILERONS  = 4,    ///< [1.0; +1.0]
            INPUT_ELEVATORS = 5,    ///< [1.0; +1.0]
            INPUT_RUDDERS   = 6,    ///< [1.0; +1.0]
            INPUT_THROTLE   = 7,    ///< [0.0; +1.0]
        };
        void motorsCallback(sensor_msgs::Joy sp_from_px4) override;
};

#endif  // SRC_MIXERS_MIXER_PX4_V_1_12_1_13070_HPP

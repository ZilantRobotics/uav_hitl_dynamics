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

#ifndef SRC_MIXERS_MIXER_PX4_V_1_14_0_13000_VTOL_QUADPLANE_HPP
#define SRC_MIXERS_MIXER_PX4_V_1_14_0_13000_VTOL_QUADPLANE_HPP

#include "base_mixer.hpp"

class PX4_V_1_14_0_Airframe_13000_to_VTOL : public BaseReverseMixer {
    public:
        using BaseReverseMixer::BaseReverseMixer;
        ~PX4_V_1_14_0_Airframe_13000_to_VTOL() final = default;
        int8_t init() override;
        ros::Subscriber servosSub;
    protected:
        enum InputAirframe : uint8_t {
            INPUT_THROTLE   = 4,    ///< [ 0.0; +1.0]
            INPUT_AILERON_1 = 5,    ///< [-1.0; +1.0]
            INPUT_AILERON_2 = 6,    ///< [-1.0; +1.0]
            INPUT_ELEVATORS = 7,    ///< [-1.0; +1.0]
            INPUT_RUDDERS   = 8,    ///< [-1.0; +1.0]
        };

        void motorsCallback(sensor_msgs::Joy msg) override;
        void servosCallback(sensor_msgs::Joy msg);
        sensor_msgs::Joy actuatorsMsg;
};

#endif  // SRC_MIXERS_MIXER_PX4_V_1_14_0_13000_VTOL_QUADPLANE_HPP

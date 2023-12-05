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

#ifndef SRC_MIXERS_BASE_MIXER_HPP
#define SRC_MIXERS_BASE_MIXER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

enum MulticopterMapping {
    MC_MOTOR_0_FRONT_RIGHT = 0,     ///< [ 0.0; +1.0]
    MC_MOTOR_1_REAR_LEFT,           ///< [ 0.0; +1.0]
    MC_MOTOR_2_FRONT_LEFT,          ///< [ 0.0; +1.0]
    MC_MOTOR_3_REAR_RIGHT,          ///< [ 0.0; +1.0]
};

enum VtolDynamicsMapping {
    VTOL_MOTOR_0_FRONT_RIGHT = 0,   ///< [ 0.0; +1.0]
    VTOL_MOTOR_1_REAR_LEFT,         ///< [ 0.0; +1.0]
    VTOL_MOTOR_2_FRONT_LEFT,        ///< [ 0.0; +1.0]
    VTOL_MOTOR_3_REAR_RIGHT,        ///< [ 0.0; +1.0]
    VTOL_AILERONS,                  ///< [-1.0; +1.0]
    VTOL_ELEVATORS,                 ///< [-1.0; +1.0]
    VTOL_RUDDERS,                   ///< [-1.0; +1.0]
    VTOL_THROTLE,                   ///< [ 0.0; +1.0]
};

class BaseReverseMixer {
    public:
        explicit BaseReverseMixer(const ros::NodeHandle& nh): _node(nh) {}
        virtual ~BaseReverseMixer() = default;
        virtual int8_t init();
        virtual void motorsCallback(sensor_msgs::Joy msg) = 0;

        ros::Publisher actuatorsPub;
        sensor_msgs::Joy sp_to_dynamics;
        ros::NodeHandle _node;
        ros::Subscriber motorsSub;

        inline static constexpr char MAPPED_ACTUATOR_TOPIC[]   = "/uav/actuators";
        inline static constexpr char MOTORS_TOPIC[]            = "/uav/actuators_raw";
        inline static constexpr char SERVOS_TOPIC[]            = "/uav/servo";
};

float clamp_float(float value, float min, float max);

#endif  // SRC_MIXERS_BASE_MIXER_HPP

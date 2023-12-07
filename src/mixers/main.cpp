/*
 * Copyright (c) 2022-2023 RaccoonLab.
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

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "mixer_babyshark.hpp"
#include "px4_v1.12.1_13070.hpp"
#include "px4_v1.14.0_13000_vtol_4_motors.hpp"
#include "px4_v1.14.0_13000_vtol_8_motors.hpp"
#include "mixer_direct.hpp"


int main(int argc, char **argv){
    ros::init(argc, argv, "mixer_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle node_handler("inno_vtol_reverse_mixer");
    std::string mixer;
    if(!node_handler.getParam("mixer", mixer)){
        ROS_ERROR("ReverseMixer: There is no `mixer` parameter.");
        return -1;
    }

    std::unique_ptr<BaseReverseMixer> reverseMixer;
    if (mixer == "babyshark_standard_vtol_mixer") {
        reverseMixer = std::make_unique<BabysharkReverseMixer>(node_handler);
    } else if (mixer == "vtol_13070_mixer") {
        reverseMixer = std::make_unique<PX4_V_1_12_1_Airframe_13070_to_VTOL>(node_handler);
    } else if (mixer == "px4_v1_14_0_vtol_13000_mixer") {
        reverseMixer = std::make_unique<PX4_V_1_14_0_Airframe_13000_4_motors>(node_handler);
    } else if (mixer == "px4_v1_14_0_vtol_13000_8_motors_mixer") {
        reverseMixer = std::make_unique<PX4_V_1_14_0_Airframe_13000_8_motors>(node_handler);
    } else if (mixer == "direct_mixer") {
        reverseMixer = std::make_unique<DirectMixer>(node_handler);
    } else {
        ROS_ERROR("ReverseMixer: Wrong `/uav/sim_params/mixer` parameter.");
        return -1;
    }
    ROS_INFO_STREAM("ReverseMixer: mixer is " << mixer.c_str());

    if (reverseMixer->init() == -1){
        ROS_ERROR("Shutdown.");
        ros::shutdown();
        return -1;
    }

    ros::spin();
    return 0;
}

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

static constexpr char MAPPED_ACTUATOR_TOPIC[]   = "/uav/actuators";
static constexpr char RAW_ACTUATOR_TOPIC[]      = "/uav/actuators_raw";

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

enum VTOL_OUTPUTS {
    VTOL_MOTOR_0 = 0,
    VTOL_MOTOR_1,
    VTOL_MOTOR_2,
    VTOL_MOTOR_3,
    AILERONS,
    ELEVATORS,
    RUDDERS,
    THROTLE,
};

class BaseReverseMixer {
    public:
        explicit BaseReverseMixer(const ros::NodeHandle& nh): _node(nh) {}
        virtual ~BaseReverseMixer() = default;
        int8_t init();
        virtual void motorsCallback(sensor_msgs::Joy msg) = 0;

        ros::Publisher mappedActuatorPub_;
        sensor_msgs::Joy actuatorMsg;
    private:
        ros::NodeHandle _node;
        ros::Subscriber motorsSub;
};

int8_t BaseReverseMixer::init() {
    motorsSub = _node.subscribe(RAW_ACTUATOR_TOPIC, 2, &BaseReverseMixer::motorsCallback, this);
    mappedActuatorPub_ = _node.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC, 5);
    return 0;
}


class BabysharkReverseMixer : public BaseReverseMixer {
    public:
        explicit BabysharkReverseMixer(const ros::NodeHandle& nh) : BaseReverseMixer(nh) {
            for (size_t channel = 0; channel < 8; channel++) {
                actuatorMsg.axes.push_back(0);
            }
        }
        ~BabysharkReverseMixer() final = default;
    protected:
        void motorsCallback(sensor_msgs::Joy msg) override;
};
void BabysharkReverseMixer::motorsCallback(sensor_msgs::Joy msg) {
    if (msg.axes.size() == 8) {
        actuatorMsg.header = msg.header;

        actuatorMsg.axes[MC_MOTOR_0] = msg.axes[BABY_SHARK_MOTOR_0];
        actuatorMsg.axes[MC_MOTOR_1] = msg.axes[BABY_SHARK_MOTOR_1];
        actuatorMsg.axes[MC_MOTOR_2] = msg.axes[BABY_SHARK_MOTOR_2];
        actuatorMsg.axes[MC_MOTOR_3] = msg.axes[BABY_SHARK_MOTOR_3];

        float roll = msg.axes[BABY_SHARK_AILERONS];
        roll = (roll < 0) ? 0.5 : 1 - roll;
        actuatorMsg.axes[VTOL_ROLL] = roll;

        float pitch = -msg.axes[BABY_SHARK_A_TAIL_LEFT] + msg.axes[BABY_SHARK_A_TAIL_RIGHT];
        pitch = (pitch < 0) ? 0.0f : pitch / 0.8f;
        actuatorMsg.axes[VTOL_PITCH] = pitch;

        float yaw = msg.axes[BABY_SHARK_A_TAIL_LEFT] + msg.axes[BABY_SHARK_A_TAIL_RIGHT];
        yaw = (yaw < 0) ? 0.0f : (1.0f - yaw) / 0.7f;
        actuatorMsg.axes[VTOL_YAW] = yaw;

        actuatorMsg.axes[VTOL_THROTTLE] = msg.axes[BABY_SHARK_PUSHER_MOTOR];
        mappedActuatorPub_.publish(actuatorMsg);
    }
}


class InnoVtolReverseMixer : public BaseReverseMixer {
    public:
        using BaseReverseMixer::BaseReverseMixer;
        ~InnoVtolReverseMixer() final = default;
    protected:
        void motorsCallback(sensor_msgs::Joy msg) override;
};
void InnoVtolReverseMixer::motorsCallback(sensor_msgs::Joy msg) {
    if (msg.axes.size() == 8) {
        if (msg.axes[4] < 0) {
            msg.axes[4] = 0.5;
        }
        msg.axes[5] = (msg.axes[5] >= 0) ? msg.axes[5] * 2.0f - 1.0f : 0.0f;
        msg.axes[6] = (msg.axes[6] >= 0) ? msg.axes[6] * 2.0f - 1.0f : 0.0f;
        msg.axes[7] = msg.axes[7] / 0.75f;
        mappedActuatorPub_.publish(msg);
    } else if (msg.axes.size() == 4) {
        msg.axes.push_back(0.5);
        msg.axes.push_back(0.0);
        msg.axes.push_back(0.0);
        msg.axes.push_back(0.0);
        mappedActuatorPub_.publish(msg);
    }
}


class DirectMixer : public BaseReverseMixer {
    public:
        explicit DirectMixer(const ros::NodeHandle& nh) : BaseReverseMixer(nh) {
            for (size_t channel = 0; channel < MAX_CHANNELS; channel++) {
                actuatorMsg.axes.push_back(0);
            }
        }
        ~DirectMixer() final = default;
    protected:
        void motorsCallback(sensor_msgs::Joy msg) override;
        static constexpr uint8_t MAX_CHANNELS = 8;
};
void DirectMixer::motorsCallback(sensor_msgs::Joy msg) {
    if (msg.axes.size() < 4) {
        return;
    }

    auto channels_amount = std::min(MAX_CHANNELS, (uint8_t)msg.axes.size());
    for (uint8_t idx = 0; idx < channels_amount; idx++) {
        actuatorMsg.axes[idx] = msg.axes[idx];
    }

    actuatorMsg.header = msg.header;
    mappedActuatorPub_.publish(actuatorMsg);
}
constexpr uint8_t DirectMixer::MAX_CHANNELS;


int main(int argc, char **argv){
    ros::init(argc, argv, "inno_vtol_reverse_mixer_node");
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
        reverseMixer = std::make_unique<InnoVtolReverseMixer>(node_handler);
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
        return 0;
    }

    ros::spin();
    return 0;
}

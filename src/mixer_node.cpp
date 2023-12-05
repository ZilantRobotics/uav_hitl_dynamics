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
static constexpr char MOTORS_TOPIC[]            = "/uav/actuators_raw";
static constexpr char SERVOS_TOPIC[]            = "/uav/servo";

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
    VTOL_MOTOR_0_FRONT_RIGHT = 0,   ///< [ 0.0; +1.0]
    VTOL_MOTOR_1_REAR_LEFT,         ///< [ 0.0; +1.0]
    VTOL_MOTOR_2_FRONT_LEFT,        ///< [ 0.0; +1.0]
    VTOL_MOTOR_3_REAR_RIGHT,        ///< [ 0.0; +1.0]
    VTOL_AILERONS,                  ///< [-1.0; +1.0]
    VTOL_ELEVATORS,                 ///< [-1.0; +1.0]
    VTOL_RUDDERS,                   ///< [-1.0; +1.0]
    VTOL_THROTLE,                   ///< [ 0.0; +1.0]
};

static float clamp_float(float value, float min, float max) {
    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    }
    return value;
}

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
};

int8_t BaseReverseMixer::init() {
    actuatorsPub = _node.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC, 5);
    motorsSub = _node.subscribe(MOTORS_TOPIC, 2, &BaseReverseMixer::motorsCallback, this);
    return 0;
}


class BabysharkReverseMixer : public BaseReverseMixer {
    public:
        explicit BabysharkReverseMixer(const ros::NodeHandle& nh) : BaseReverseMixer(nh) {
            for (size_t channel = 0; channel < 8; channel++) {
                sp_to_dynamics.axes.push_back(0);
            }
        }
        ~BabysharkReverseMixer() final = default;
    protected:
        void motorsCallback(sensor_msgs::Joy msg) override;
};
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
int8_t PX4_V_1_12_1_Airframe_13070_to_VTOL::init() {
    actuatorsPub = _node.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC, 5);
    motorsSub = _node.subscribe(MOTORS_TOPIC, 2, &PX4_V_1_12_1_Airframe_13070_to_VTOL::motorsCallback, this);

    sp_to_dynamics.axes.push_back(0.0f);
    sp_to_dynamics.axes.push_back(0.0f);
    sp_to_dynamics.axes.push_back(0.0f);
    sp_to_dynamics.axes.push_back(0.0f);
    sp_to_dynamics.axes.push_back(0.0f);
    sp_to_dynamics.axes.push_back(0.0f);
    sp_to_dynamics.axes.push_back(0.0f);
    sp_to_dynamics.axes.push_back(0.0f);

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
int8_t PX4_V_1_14_0_Airframe_13000_to_VTOL::init() {
    actuatorsPub = _node.advertise<sensor_msgs::Joy>(MAPPED_ACTUATOR_TOPIC, 5);

    motorsSub = _node.subscribe(MOTORS_TOPIC, 2, &PX4_V_1_14_0_Airframe_13000_to_VTOL::motorsCallback, this);
    servosSub = _node.subscribe(SERVOS_TOPIC, 2, &PX4_V_1_14_0_Airframe_13000_to_VTOL::servosCallback, this);

    for (uint_fast8_t idx = 0; idx < 8; idx++) {
        actuatorsMsg.axes.push_back(0);
    }

    return 0;
}
void PX4_V_1_14_0_Airframe_13000_to_VTOL::motorsCallback(sensor_msgs::Joy msg) {
    auto& axes = actuatorsMsg.axes;
    if (msg.axes.size() >= 4) {
        axes[VTOL_MOTOR_0_FRONT_RIGHT] = clamp_float(msg.axes[0], 0.0, 1.0);
        axes[VTOL_MOTOR_1_REAR_LEFT] = clamp_float(msg.axes[1], 0.0, 1.0);
        axes[VTOL_MOTOR_2_FRONT_LEFT] = clamp_float(msg.axes[2], 0.0, 1.0);
        axes[VTOL_MOTOR_3_REAR_RIGHT] = clamp_float(msg.axes[3], 0.0, 1.0);
    }
    if (msg.axes.size() >= 5) {
        axes[VTOL_THROTLE] = clamp_float(msg.axes[INPUT_THROTLE], 0.0, 1.0);
    }
    if (msg.axes.size() >= 9) {
        if (abs(msg.axes[INPUT_AILERON_2]) < 0.001 &&
                abs(msg.axes[INPUT_ELEVATORS]) < 0.001 &&
                abs(msg.axes[INPUT_RUDDERS]) < 0.001) {
            axes[VTOL_AILERONS] = 0.0;
            axes[VTOL_ELEVATORS] = 0.0;
            axes[VTOL_RUDDERS] = 0.0;
        } else {
            axes[VTOL_AILERONS] = 2.0 * clamp_float(msg.axes[INPUT_AILERON_2], 0.0, 1.0) - 1.0;
            axes[VTOL_ELEVATORS] = 2.0 * clamp_float(msg.axes[INPUT_ELEVATORS], 0.0, 1.0) - 1.0;
            axes[VTOL_RUDDERS] = 2.0 * clamp_float(msg.axes[INPUT_RUDDERS], 0.0, 1.0) - 1.0;
        }
    }

    actuatorsPub.publish(actuatorsMsg);
}
void PX4_V_1_14_0_Airframe_13000_to_VTOL::servosCallback(sensor_msgs::Joy msg) {
    ///< ignore left aileron msg.axes[0] here
    actuatorsMsg.axes[VTOL_AILERONS] = clamp_float(msg.axes[1], -1.0, 1.0);
    actuatorsMsg.axes[VTOL_ELEVATORS] = clamp_float(msg.axes[2], -1.0, 1.0);
    actuatorsMsg.axes[VTOL_RUDDERS] = clamp_float(msg.axes[3], -1.0, 1.0);
}


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
        static constexpr uint8_t MAX_CHANNELS = 8;
};
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
constexpr uint8_t DirectMixer::MAX_CHANNELS;

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
        reverseMixer = std::make_unique<PX4_V_1_14_0_Airframe_13000_to_VTOL>(node_handler);
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

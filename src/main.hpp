/*
 * Copyright (c) 2020-2023 RaccoonLab.
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
 * Authors: Dmitry Ponomarev <ponomarevda96@gmail.com>
 *          Roman Fedorenko <frontwise@gmail.com>
 */

#ifndef SRC_MAIN_HPP
#define SRC_MAIN_HPP

#include <thread>
#include <random>
#include <geographiclib_conversions/geodetic_conv.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/UInt8.h>

#include "uavDynamicsSimBase.hpp"
#include "dynamics.hpp"
#include "actuators.hpp"
#include "sensors.hpp"
#include "scenarios.hpp"
#include "logger.hpp"
#include "rviz_visualization.hpp"


/**
 * @brief UAV Dynamics class used for dynamics, IMU, and angular rate control simulation node
 */
class Uav_Dynamics {
    public:
        explicit Uav_Dynamics(ros::NodeHandle nh);
        int8_t init();

    private:
        int8_t getParamsFromRos();
        int8_t initDynamicsSimulator();
        int8_t initSensors();
        int8_t initCalibration();
        int8_t startClockAndThreads();

        // Simulator
        ros::NodeHandle _node;
        std::shared_ptr<UavDynamicsSimBase> uavDynamicsSim_;
        ros::Publisher clockPub_;

        ros::Time currentTime_;
        double dt_secs_ = 1.0f/960.;
        double clockScale_ = 1.0;
        bool useSimTime_;

        std::vector<double> initPose_{7};
        std::vector<double> _wind_ned{3};

        DynamicsInfo info;

        Actuators _actuators;
        Sensors _sensors;
        RvizVisualizator _rviz_visualizator;
        ScenarioManager _scenarioManager;
        StateLogger _logger;

        // Calibration
        ros::Subscriber calibrationSub_;
        UavDynamicsSimBase::SimMode_t calibrationType_{UavDynamicsSimBase::SimMode_t::NORMAL};
        void calibrationCallback(std_msgs::UInt8 msg);

        // Diagnostic
        uint64_t dynamicsCounter_;
        uint64_t rosPubCounter_;

        // Timer and threads
        ros::WallTimer simulationLoopTimer_;
        std::thread proceedDynamicsTask;
        std::thread publishToRosTask;
        std::thread diagnosticTask;

        void simulationLoopTimerCallback(const ros::WallTimerEvent& event);
        void proceedDynamics(double period);
        void publishToRos(double period);
        void performLogging(double period);

        const float ROS_PUB_PERIOD_SEC = 0.05f;
};

#endif  // SRC_MAIN_HPP

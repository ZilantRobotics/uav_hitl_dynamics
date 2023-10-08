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

#include "main.hpp"
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Time.h>

#include "quadcopter.hpp"
#include "octocopter.hpp"
#include "vtolDynamicsSim.hpp"
#include "cs_converter.hpp"


int main(int argc, char **argv){
    ros::init(argc, argv, "uav_dynamics_node");
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle node_handler("inno_dynamics_sim");
    Uav_Dynamics uav_dynamics_node(node_handler);
    if(uav_dynamics_node.init() == -1){
        ROS_ERROR("Shutdown.");
        ros::shutdown();
        return -1;
    }

    ros::spin();
    return 0;
}


Uav_Dynamics::Uav_Dynamics(ros::NodeHandle nh) :
    _node(nh),
    _sensors(&nh),
    _rviz_visualizator(_node),
    _scenarioManager(_node, _actuators, _sensors),
    _logger(_actuators, _sensors, info){
}


/**
 * @return -1 if error occured, else 0
 */
int8_t Uav_Dynamics::init(){
    if(getParamsFromRos() == -1){
        return -1;
    }else if(initDynamicsSimulator() == -1){
        return -1;
    }else if(initSensors() == -1){
        return -1;
    }else if(initCalibration() == -1){
        return -1;
    }else if(_rviz_visualizator.init(uavDynamicsSim_) == -1){
        return -1;
    }else if(startClockAndThreads() == -1){
        return -1;
    }

    return 0;
}

int8_t Uav_Dynamics::getParamsFromRos(){
    const std::string SIM_PARAMS_PATH = "/uav/sim_params/";
    if(!ros::param::get(SIM_PARAMS_PATH + "use_sim_time",       useSimTime_ )           ||
       !_node.getParam("vehicle",                               info.vehicleName)       ||
       !_node.getParam("dynamics",                              info.dynamicsName)      ||
       !ros::param::get(SIM_PARAMS_PATH + "init_pose",          initPose_)){
        ROS_ERROR("Dynamics: There is no at least one of required simulator parameters.");
        return -1;
    }
    return 0;
}

int8_t Uav_Dynamics::initDynamicsSimulator(){
    if(info.dynamicsName == "quadcopter"){
        info.dynamicsType = DynamicsType::QUADCOPTER;
        uavDynamicsSim_ = std::make_shared<QuadcopterDynamics>();
        info.notation = DynamicsNotation_t::ROS_ENU_FLU;
    }else if(info.dynamicsName == "octorotor"){
        info.dynamicsType = DynamicsType::OCTOCOPTER;
        uavDynamicsSim_ = std::make_shared<OctocopterDynamics>();
        info.notation = DynamicsNotation_t::ROS_ENU_FLU;
    }else if(info.dynamicsName == "vtol_dynamics"){
        uavDynamicsSim_ = std::make_shared<InnoVtolDynamicsSim>();
        info.dynamicsType = DynamicsType::VTOL;
        info.notation = DynamicsNotation_t::PX4_NED_FRD;
    }else{
        ROS_ERROR("Dynamics type with name \"%s\" is not exist.", info.dynamicsName.c_str());
        return -1;
    }

    if(info.vehicleName == "innopolis_vtol"){
        info.vehicleType = VehicleType::INNOPOLIS_VTOL;
    }else if(info.vehicleName == "iris"){
        info.vehicleType = VehicleType::IRIS;
    }else{
        ROS_ERROR("Wrong vehicle. It should be 'innopolis_vtol' or 'iris'");
        return -1;
    }

    if(uavDynamicsSim_ == nullptr || uavDynamicsSim_->init() == -1){
        ROS_ERROR("Can't init uav dynamics sim. Shutdown.");
        return -1;
    }

    Eigen::Vector3d initPosition(initPose_.at(0), initPose_.at(1), initPose_.at(2));
    Eigen::Quaterniond initAttitude(initPose_.at(6), initPose_.at(3), initPose_.at(4), initPose_.at(5));
    initAttitude.normalize();
    uavDynamicsSim_->setInitialPosition(initPosition, initAttitude);

    return 0;
}

int8_t Uav_Dynamics::initSensors(){
    _actuators.init(_node);
    _scenarioManager.init();
    _logger.init(clockScale_, dt_secs_);
    return _sensors.init(uavDynamicsSim_);
}

int8_t Uav_Dynamics::initCalibration(){
    calibrationSub_ = _node.subscribe("/uav/calibration", 1, &Uav_Dynamics::calibrationCallback, this);
    return 0;
}

int8_t Uav_Dynamics::startClockAndThreads(){
    ros::Duration(0.1).sleep();
    if(useSimTime_){
        clockPub_ = _node.advertise<rosgraph_msgs::Clock>("/clock", 1);
        rosgraph_msgs::Clock clock_time;
        clock_time.clock = currentTime_;
        clockPub_.publish(clock_time);
    }else{
        // Get the current time if we are using wall time. Otherwise, use 0 as initial clock.
        currentTime_ = ros::Time::now();
    }


    simulationLoopTimer_ = _node.createWallTimer(ros::WallDuration(dt_secs_/clockScale_),
                                                 &Uav_Dynamics::simulationLoopTimerCallback,
                                                 this);
    simulationLoopTimer_.start();

    proceedDynamicsTask = std::thread(&Uav_Dynamics::proceedDynamics, this, dt_secs_);
    proceedDynamicsTask.detach();

    publishToRosTask = std::thread(&Uav_Dynamics::publishToRos, this, ROS_PUB_PERIOD_SEC);
    publishToRosTask.detach();

    diagnosticTask = std::thread(&Uav_Dynamics::performLogging, this, 1.0);
    diagnosticTask.detach();

    return 0;
}

/**
 * @brief Main Simulator loop
 * @param event Wall clock timer event
 */
void Uav_Dynamics::simulationLoopTimerCallback(const ros::WallTimerEvent&){
    if (useSimTime_){
        currentTime_ += ros::Duration(dt_secs_);
        rosgraph_msgs::Clock clock_time;
        clock_time.clock = currentTime_;
        clockPub_.publish(clock_time);
    } else {
        ros::Time loopStartTime = ros::Time::now();
        dt_secs_ = (loopStartTime - currentTime_).toSec();
        currentTime_ = loopStartTime;
    }
}

void Uav_Dynamics::performLogging(double periodSec){
    while(ros::ok()){
        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::seconds(int(periodSec * clockScale_));

        std::stringstream logStream;
        auto pose = uavDynamicsSim_->getVehiclePosition();
        _logger.createStringStream(logStream, pose, dynamicsCounter_, rosPubCounter_, periodSec);
        dynamicsCounter_ = 0;
        rosPubCounter_ = 0;


        ROS_INFO_STREAM(logStream.str());
        fflush(stdout);
        std::this_thread::sleep_until(crnt_time + sleed_period);
    }
}

// The sequence of steps for lockstep are:
// The simulation sends a sensor message HIL_SENSOR including a timestamp time_usec to update
// the sensor state and time of PX4.
// PX4 receives this and does one iteration of state estimation, controls, etc. and eventually
// sends an actuator message HIL_ACTUATOR_CONTROLS.
// The simulation waits until it receives the actuator/motor message, then simulates the physics
// and calculates the next sensor message to send to PX4 again.
// The system starts with a "freewheeling" period where the simulation sends sensor messages
// including time and therefore runs PX4 until it has initialized and responds with an actautor
// message.
// But instead of waiting actuators cmd, we will wait for an arming
void Uav_Dynamics::proceedDynamics(double periodSec){
    while(ros::ok()){
        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::milliseconds(int(1000 * periodSec * clockScale_));
        auto time_point = crnt_time + sleed_period;
        dynamicsCounter_++;

        if(calibrationType_ != UavDynamicsSimBase::SimMode_t::NORMAL){
            uavDynamicsSim_->calibrate(calibrationType_);
        }else if(_actuators.getArmingStatus() != ArmingStatus::DISARMED){
            static auto crnt_time = std::chrono::system_clock::now();
            auto prev_time = crnt_time;
            crnt_time = std::chrono::system_clock::now();
            auto time_dif_sec = static_cast<double>((crnt_time - prev_time).count()) * 1e-9;

            ///< prevent big time jumping
            const double MAX_TIME_DIFF_SEC = 10 * periodSec;
            if (time_dif_sec > MAX_TIME_DIFF_SEC) {
                ROS_ERROR_STREAM_THROTTLE(1, "Time jumping: " << time_dif_sec << " seconds.");
                time_dif_sec = MAX_TIME_DIFF_SEC;
            }

            uavDynamicsSim_->process(time_dif_sec, _actuators.actuators, true);
        }else{
            uavDynamicsSim_->land();
        }

        _sensors.publishStateToCommunicator((uint8_t)info.notation);

        std::this_thread::sleep_until(time_point);
    }
}

void Uav_Dynamics::publishToRos(double period){
    while(ros::ok()){
        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::microseconds(int(1000000 * period * clockScale_));
        auto time_point = crnt_time + sleed_period;
        rosPubCounter_++;

        _rviz_visualizator.publishTf((uint8_t)info.notation);

        static auto next_time = std::chrono::system_clock::now();
        if(crnt_time > next_time){
            if (info.dynamicsType == DynamicsType::VTOL) {
                _rviz_visualizator.publish((uint8_t)info.notation);
            }
            next_time += std::chrono::milliseconds(int(50));
        }

        std::this_thread::sleep_until(time_point);
    }
}

void Uav_Dynamics::calibrationCallback(std_msgs::UInt8 msg){
    if(calibrationType_ != static_cast<UavDynamicsSimBase::SimMode_t>(msg.data)){
        ROS_INFO_STREAM_THROTTLE(1, "calibration type: " << msg.data + 0);
    }
    calibrationType_ = static_cast<UavDynamicsSimBase::SimMode_t>(msg.data);
}

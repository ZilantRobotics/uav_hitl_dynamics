/**
 * @file uavDynamicsNode.hpp
 * @author Dmitry Ponomarev
 * @author Roman Fedorenko
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Header file for UAV dynamics, IMU, and angular rate control simulation node
 */

#ifndef UAV_DYNAMICS_HPP
#define UAV_DYNAMICS_HPP

#include <thread>
#include <random>
#include <geographiclib_conversions/geodetic_conv.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

#include "uavDynamicsSimBase.hpp"
#include "sensors.hpp"
#include "rviz_visualization.hpp"


/**
 * @brief UAV Dynamics class used for dynamics, IMU, and angular rate control simulation node
 */
class Uav_Dynamics {
    public:
        explicit Uav_Dynamics(ros::NodeHandle nh);
        int8_t init();

        enum DynamicsNotation_t{
            PX4_NED_FRD = 0,
            ROS_ENU_FLU = 1,
        };

    private:
        int8_t getParamsFromRos();
        int8_t initDynamicsSimulator();
        int8_t initSensors();
        int8_t initCalibration();
        int8_t startClockAndThreads();

        /// @name Simulator
        //@{
        ros::NodeHandle node_;
        std::shared_ptr<UavDynamicsSimBase> uavDynamicsSim_;
        ros::Publisher clockPub_;

        ros::Time currentTime_;
        double dt_secs_ = 1.0f/960.;
        double clockScale_ = 1.0;
        bool useSimTime_;

        std::vector<double> initPose_;

        enum DynamicsType{
            DYNAMICS_FLIGHTGOGGLES_MULTICOPTER = 0,
            DYNAMICS_INNO_VTOL,
        };
        enum VehicleType{
            VEHICLE_IRIS = 0,
            VEHICLE_INNOPOLIS_VTOL,
        };

        DynamicsType dynamicsType_;
        VehicleType vehicleType_;

        std::string vehicleName_;
        std::string dynamicsTypeName_;
        //@}


        /// @name Communication with PX4
        //@{
        ros::Subscriber actuatorsSub_;
        std::vector<double> actuators_;
        uint64_t lastActuatorsTimestampUsec_;
        uint64_t prevActuatorsTimestampUsec_;
        uint64_t maxDelayUsec_;
        void actuatorsCallback(sensor_msgs::Joy::Ptr msg);

        ros::Subscriber armSub_;
        bool armed_ = false;
        void armCallback(std_msgs::Bool msg);

        ros::Subscriber scenarioSub_;
        uint8_t _scenarioType = 0;
        void scenarioCallback(std_msgs::UInt8 msg);

        Sensors _sensors;
        RvizVisualizator _rviz_visualizator;
        //@}

        /// @name Calibration
        //@{
        ros::Subscriber calibrationSub_;
        UavDynamicsSimBase::SimMode_t calibrationType_{UavDynamicsSimBase::SimMode_t::NORMAL};
        void calibrationCallback(std_msgs::UInt8 msg);
        //@}

        /// @name Diagnostic
        //@{
        uint64_t actuatorsMsgCounter_ = 0;
        uint64_t dynamicsCounter_;
        uint64_t rosPubCounter_;
        //@}

        /// @name Timer and threads
        //@{
        ros::WallTimer simulationLoopTimer_;
        std::thread proceedDynamicsTask;
        std::thread publishToRosTask;
        std::thread diagnosticTask;

        void simulationLoopTimerCallback(const ros::WallTimerEvent& event);
        void proceedDynamics(double period);
        void publishToRos(double period);
        void performLogging(double period);

        const float ROS_PUB_PERIOD_SEC = 0.05f;
        //@}

        DynamicsNotation_t _dynamicsNotation;
};

#endif

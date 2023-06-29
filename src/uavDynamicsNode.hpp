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

#include <std_msgs/UInt8.h>

#include "uavDynamicsSimBase.hpp"
#include "actuators.hpp"
#include "sensors.hpp"
#include "scenarios.hpp"
#include "rviz_visualization.hpp"

enum class DynamicsType{
    FLIGHTGOGGLES_MULTICOPTER = 0,
    INNO_VTOL,
};

enum class VehicleType{
    IRIS = 0,
    INNOPOLIS_VTOL,
};

enum class DynamicsNotation_t{
    PX4_NED_FRD = 0,
    ROS_ENU_FLU = 1,
};


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

        DynamicsType dynamicsType_;
        VehicleType vehicleType_;
        DynamicsNotation_t _dynamicsNotation;

        std::string vehicleName_;
        std::string dynamicsTypeName_;


        // Communication with PX4
        Actuators _actuators;
        Sensors _sensors;
        RvizVisualizator _rviz_visualizator;
        ScenarioManager _scenarioManager;

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

#endif

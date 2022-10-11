/**
 * @file innopolis_vtol_dynamics_node.hpp
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
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/Marker.h>

#include "uavDynamicsSimBase.hpp"
#include "sensors.hpp"


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
        int8_t initRvizVisualizationMarkers();
        int8_t startClockAndThreads();

        /// @name Simulator
        //@{
        ros::NodeHandle node_;
        UavDynamicsSimBase* uavDynamicsSim_;
        ros::Publisher clockPub_;

        ros::Time currentTime_;
        double dt_secs_ = 1.0f/960.;
        bool useAutomaticClockscale_ = false;
        double clockScale_ = 1.0;
        double actualFps_  = -1;
        bool useSimTime_;

        double latRef_;
        double lonRef_;
        double altRef_;
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

        geodetic_converter::GeodeticConverter geodeticConverter_;
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

        AttitudeSensor attitudeSensor_;
        ImuSensor imuSensor_;
        VelocitySensor velocitySensor_;
        MagSensor magSensor_;
        RawAirDataSensor rawAirDataSensor_;
        TemperatureSensor temperatureSensor_;
        PressureSensor pressureSensor_;
        EscStatusSensor escStatusSensor_;
        GpsSensor gpsSensor_;
        IceStatusSensor iceStatusSensor_;
        FuelTankSensor fuelTankSensor_;
        BatteryInfoSensor batteryInfoSensor_;

        bool isEscStatusEnabled_;
        bool isIceStatusEnabled_;
        bool isFuelTankEnabled_;
        bool isBatteryInfoEnabled_;

        void publishStateToCommunicator();
        //@}

        /// @name Calibration
        //@{
        ros::Subscriber calibrationSub_;
        UavDynamicsSimBase::CalibrationType_t calibrationType_ = UavDynamicsSimBase::WORK_MODE;
        void calibrationCallback(std_msgs::UInt8 msg);
        //@}

        /// @name Diagnostic
        //@{
        uint64_t actuatorsMsgCounter_ = 0;
        uint64_t dynamicsCounter_;
        uint64_t rosPubCounter_;
        //@}

        /// @name Visualization (Markers and tf)
        //@{
        tf2_ros::TransformBroadcaster tfPub_;

        visualization_msgs::Marker arrowMarkers_;

        ros::Publisher totalForcePub_;
        ros::Publisher aeroForcePub_;
        ros::Publisher motorsForcesPub_[5];
        ros::Publisher liftForcePub_;
        ros::Publisher drugForcePub_;
        ros::Publisher sideForcePub_;

        ros::Publisher totalMomentPub_;
        ros::Publisher aeroMomentPub_;
        ros::Publisher controlSurfacesMomentPub_;
        ros::Publisher aoaMomentPub_;
        ros::Publisher motorsMomentsPub_[5];

        ros::Publisher velocityPub_;

        void initMarkers();
        visualization_msgs::Marker& makeArrow(const Eigen::Vector3d& vector3D,
                                              const Eigen::Vector3d& rgbColor,
                                              const char* frameId);
        void publishMarkers();
        void publishState();
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

        const float ROS_PUB_PERIOD_SEC = 0.05;
        //@}

        enum DynamicsNotation_t{
            PX4_NED_FRD = 0,
            ROS_ENU_FLU = 1,
        };
        DynamicsNotation_t dynamicsNotation_;
};

#endif

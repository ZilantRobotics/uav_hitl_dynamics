/**
 * @file uavDynamicsNode.cpp
 * @author Dmitry Ponomarev
 * @author Roman Fedorenko
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Implementation of UAV dynamics, IMU, and angular rate control simulation node
 */

#include "uavDynamicsNode.hpp"
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Time.h>

#include "flightgogglesDynamicsSim.hpp"
#include "vtolDynamicsSim.hpp"
#include "cs_converter.hpp"
#include "sensors_isa_model.hpp"


static char GLOBAL_FRAME_ID[] = "world";
static char UAV_FRAME_ID[] = "uav/enu";
static char UAV_FIXED_FRAME_ID[] = "uav/com";
const std::string MOTOR_NAMES[5] = {"motor0", "motor1", "motor2", "motor3", "ICE"};


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
    node_(nh),
    actuators_(8, 0.),
    initPose_(7),
    _sensors(&nh){
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
    }else if(initRvizVisualizationMarkers() == -1){
        return -1;
    }else if(startClockAndThreads() == -1){
        return -1;
    }

    return 0;
}

int8_t Uav_Dynamics::getParamsFromRos(){
    const std::string SIM_PARAMS_PATH = "/uav/sim_params/";
    if(!ros::param::get(SIM_PARAMS_PATH + "use_sim_time",       useSimTime_ )           ||
       !ros::param::get(SIM_PARAMS_PATH + "lat_ref",            latRef_)                ||
       !ros::param::get(SIM_PARAMS_PATH + "lon_ref",            lonRef_)                ||
       !ros::param::get(SIM_PARAMS_PATH + "alt_ref",            altRef_)                ||
       !node_.getParam("vehicle",                               vehicleName_)           ||
       !node_.getParam("dynamics",                              dynamicsTypeName_)      ||
       !ros::param::get(SIM_PARAMS_PATH + "init_pose",          initPose_)){
        ROS_ERROR("Dynamics: There is no at least one of required simulator parameters.");
        return -1;
    }
    return 0;
}

int8_t Uav_Dynamics::initDynamicsSimulator(){
    const char DYNAMICS_NAME_FLIGHTGOGGLES[] = "flightgoggles_multicopter";
    const char DYNAMICS_NAME_INNO_VTOL[] = "inno_vtol";
    const char VEHICLE_NAME_INNOPOLIS_VTOL[] = "innopolis_vtol";
    const char VEHICLE_NAME_IRIS[] = "iris";
    if(dynamicsTypeName_ == DYNAMICS_NAME_FLIGHTGOGGLES){
        dynamicsType_ = DYNAMICS_FLIGHTGOGGLES_MULTICOPTER;
        uavDynamicsSim_ = new FlightgogglesDynamics;
        dynamicsNotation_ = ROS_ENU_FLU;
    }else if(dynamicsTypeName_ == DYNAMICS_NAME_INNO_VTOL){
        uavDynamicsSim_ = new InnoVtolDynamicsSim;
        dynamicsType_ = DYNAMICS_INNO_VTOL;
        dynamicsNotation_ = PX4_NED_FRD;
    }else{
        ROS_ERROR("Dynamics type with name \"%s\" is not exist.", dynamicsTypeName_.c_str());
        return -1;
    }

    if(vehicleName_ == VEHICLE_NAME_INNOPOLIS_VTOL){
        vehicleType_ = VEHICLE_INNOPOLIS_VTOL;
    }else if(vehicleName_ == VEHICLE_NAME_IRIS){
        vehicleType_ = VEHICLE_IRIS;
    }else{
        ROS_ERROR("Wrong vehicle. It should be 'innopolis_vtol' or 'iris'");
        return -1;
    }

    if(uavDynamicsSim_ == nullptr || uavDynamicsSim_->init() == -1){
        ROS_ERROR("Can't init uav dynamics sim. Shutdown.");
        return -1;
    }
    geodeticConverter_.initialiseReference(latRef_, lonRef_, altRef_);

    Eigen::Vector3d initPosition(initPose_.at(0), initPose_.at(1), initPose_.at(2));
    Eigen::Quaterniond initAttitude(initPose_.at(6), initPose_.at(3), initPose_.at(4), initPose_.at(5));
    initAttitude.normalize();
    uavDynamicsSim_->setInitialPosition(initPosition, initAttitude);

    return 0;
}

int8_t Uav_Dynamics::initSensors(){
    actuatorsSub_ = node_.subscribe("/uav/actuators", 1, &Uav_Dynamics::actuatorsCallback, this);
    armSub_ = node_.subscribe("/uav/arm", 1, &Uav_Dynamics::armCallback, this);
    scenarioSub_ = node_.subscribe("/uav/scenario", 1, &Uav_Dynamics::scenarioCallback, this);
    return _sensors.init();
}

int8_t Uav_Dynamics::initCalibration(){
    calibrationSub_ = node_.subscribe("/uav/calibration", 1, &Uav_Dynamics::calibrationCallback, this);
    return 0;
}

int8_t Uav_Dynamics::initRvizVisualizationMarkers(){
    initMarkers();
    totalForcePub_ = node_.advertise<visualization_msgs::Marker>("/uav/Ftotal", 1);
    aeroForcePub_ = node_.advertise<visualization_msgs::Marker>("/uav/Faero", 1);
    motorsForcesPub_[0] = node_.advertise<visualization_msgs::Marker>("/uav/Fmotor0", 1);
    motorsForcesPub_[1] = node_.advertise<visualization_msgs::Marker>("/uav/Fmotor1", 1);
    motorsForcesPub_[2] = node_.advertise<visualization_msgs::Marker>("/uav/Fmotor2", 1);
    motorsForcesPub_[3] = node_.advertise<visualization_msgs::Marker>("/uav/Fmotor3", 1);
    motorsForcesPub_[4] = node_.advertise<visualization_msgs::Marker>("/uav/Fmotor4", 1);
    liftForcePub_ = node_.advertise<visualization_msgs::Marker>("/uav/liftForce", 1);
    drugForcePub_ = node_.advertise<visualization_msgs::Marker>("/uav/drugForce", 1);
    sideForcePub_ = node_.advertise<visualization_msgs::Marker>("/uav/sideForce", 1);

    totalMomentPub_ = node_.advertise<visualization_msgs::Marker>("/uav/Mtotal", 1);
    aeroMomentPub_ = node_.advertise<visualization_msgs::Marker>("/uav/Maero", 1);
    controlSurfacesMomentPub_ = node_.advertise<visualization_msgs::Marker>("/uav/McontrolSurfaces", 1);
    aoaMomentPub_ = node_.advertise<visualization_msgs::Marker>("/uav/Maoa", 1);
    motorsMomentsPub_[0] = node_.advertise<visualization_msgs::Marker>("/uav/Mmotor0", 1);
    motorsMomentsPub_[1] = node_.advertise<visualization_msgs::Marker>("/uav/Mmotor1", 1);
    motorsMomentsPub_[2] = node_.advertise<visualization_msgs::Marker>("/uav/Mmotor2", 1);
    motorsMomentsPub_[3] = node_.advertise<visualization_msgs::Marker>("/uav/Mmotor3", 1);
    motorsMomentsPub_[4] = node_.advertise<visualization_msgs::Marker>("/uav/Mmotor4", 1);

    velocityPub_ = node_.advertise<visualization_msgs::Marker>("/uav/linearVelocity", 1);

    return 0;
}

int8_t Uav_Dynamics::startClockAndThreads(){
    ros::Duration(0.1).sleep();
    if(useSimTime_){
        clockPub_ = node_.advertise<rosgraph_msgs::Clock>("/clock", 1);
        rosgraph_msgs::Clock clock_time;
        clock_time.clock = currentTime_;
        clockPub_.publish(clock_time);
    }else{
        // Get the current time if we are using wall time. Otherwise, use 0 as initial clock.
        currentTime_ = ros::Time::now();
    }


    simulationLoopTimer_ = node_.createWallTimer(ros::WallDuration(dt_secs_/clockScale_),
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
void Uav_Dynamics::simulationLoopTimerCallback(const ros::WallTimerEvent& event){
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

std::string COLOR_RED = "\033[1;31m";
std::string COLOR_BOLD = "\033[1;29m";
std::string COLOR_TAIL = "\033[0m";

void logAddToStream(std::stringstream& logStream, bool is_ok, std::string& newData) {
    if(is_ok){
        logStream << COLOR_RED << newData << COLOR_TAIL;
    }else{
        logStream << newData;
    }
}

void logAddBoldStringToStream(std::stringstream& logStream, const char* newData) {
    logStream << COLOR_BOLD << newData << COLOR_TAIL;
}

void Uav_Dynamics::performLogging(double periodSec){
    while(ros::ok()){
        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::seconds(int(periodSec * clockScale_));

        std::stringstream logStream;
        logStream << dynamicsTypeName_.c_str() << ": " << "time elapsed: " << periodSec << " secs. ";

        float dynamicsCompleteness = dynamicsCounter_ * dt_secs_ / (clockScale_ * periodSec);
        std::string dyn_str = "dyn=" + std::to_string(dynamicsCompleteness);
        logAddToStream(logStream, dynamicsCompleteness >= 0.9, dyn_str);
        logStream << ", ";
        dynamicsCounter_ = 0;

        float rosPubCompleteness = rosPubCounter_ * ROS_PUB_PERIOD_SEC / (clockScale_ * periodSec);
        std::string ros_pub_str = "ros_pub=" + std::to_string(rosPubCompleteness);
        logAddToStream(logStream, rosPubCompleteness >= 0.9, ros_pub_str);
        logStream << ", ";
        rosPubCounter_ = 0;

        std::string actuator_str;
        bool is_actuator_ok = actuatorsMsgCounter_ > 100 && maxDelayUsec_ < 20000 && maxDelayUsec_ != 0;
        logAddToStream(logStream, is_actuator_ok, actuator_str);
        logStream << " us.\n";
        actuatorsMsgCounter_ = 0;
        maxDelayUsec_ = 0;

        logAddBoldStringToStream(logStream, "mc");
        logStream << std::setprecision(2) << std::fixed << " ["
                  << actuators_[0] << ", "
                  << actuators_[1] << ", "
                  << actuators_[2] << ", "
                  << actuators_[3] << "] ";

        if(vehicleType_ == VEHICLE_INNOPOLIS_VTOL){
            logAddBoldStringToStream(logStream, "fw rpy");
            logStream << " [" << actuators_[4] << ", "
                              << actuators_[5] << ", "
                              << actuators_[6] << "]";
            logAddBoldStringToStream(logStream, " throttle");
            logStream << " [" << actuators_[7] << "] ";
        }

        auto pose = uavDynamicsSim_->getVehiclePosition();
        auto enuPosition = (dynamicsNotation_ == PX4_NED_FRD) ? Converter::nedToEnu(pose) : pose;
        logAddBoldStringToStream(logStream, "enu pose");
        logStream << std::setprecision(1) << std::fixed << " ["
                  << enuPosition[0] << ", "
                  << enuPosition[1] << ", "
                  << enuPosition[2] << "].";

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

        if(calibrationType_ != UavDynamicsSimBase::CalibrationType_t::WORK_MODE){
            uavDynamicsSim_->calibrate(calibrationType_);
        }else if(armed_){
            static auto crnt_time = std::chrono::system_clock::now();
            auto prev_time = crnt_time;
            crnt_time = std::chrono::system_clock::now();
            auto time_dif_sec = (crnt_time - prev_time).count() / 1000000000.0;

            ///< prevent big time jumping
            const double MAX_TIME_DIFF_SEC = 10 * periodSec;
            if (time_dif_sec > MAX_TIME_DIFF_SEC) {
                ROS_ERROR_STREAM_THROTTLE(1, "Time jumping: " << time_dif_sec << " seconds.");
                time_dif_sec = MAX_TIME_DIFF_SEC;
            }

            uavDynamicsSim_->process(time_dif_sec, actuators_, true);
        }else{
            uavDynamicsSim_->land();
        }

        publishStateToCommunicator();

        std::this_thread::sleep_until(time_point);
    }
}

/**
 * @note Different simulators return data in different notation (PX4 or ROS)
 * But we must publish only in PX4 notation
 */
void Uav_Dynamics::publishStateToCommunicator(){
    // 1. Get data from simulator
    Eigen::Vector3d position, linVel, acc, gyro, angVel;
    Eigen::Quaterniond attitude;
    position = uavDynamicsSim_->getVehiclePosition();
    linVel = uavDynamicsSim_->getVehicleVelocity();
    uavDynamicsSim_->getIMUMeasurement(acc, gyro);
    angVel = uavDynamicsSim_->getVehicleAngularVelocity();
    attitude = uavDynamicsSim_->getVehicleAttitude();

    // 2. Convert them to appropriate CS
    Eigen::Vector3d gpsPosition, enuPosition, linVelNed, accFrd, gyroFrd, angVelFrd;
    Eigen::Quaterniond attitudeFrdToNed;
    if(dynamicsNotation_ == PX4_NED_FRD){
        enuPosition = Converter::nedToEnu(position);
        linVelNed = linVel;
        accFrd = acc;
        gyroFrd = gyro;
        angVelFrd = angVel;
        attitudeFrdToNed = attitude;
    }else{
        enuPosition = position;
        linVelNed =  Converter::enuToNed(linVel);
        accFrd = Converter::fluToFrd(acc);
        gyroFrd = Converter::fluToFrd(gyro);
        angVelFrd = Converter::fluToFrd(angVel);
        attitudeFrdToNed = Converter::fluEnuToFrdNed(attitude);
    }
    geodeticConverter_.enu2Geodetic(enuPosition[0], enuPosition[1], enuPosition[2],
                                    &gpsPosition[0], &gpsPosition[1], &gpsPosition[2]);

    // 3. Calculate temperature, abs pressure and diff pressure using ISA model
    float temperatureKelvin, absPressureHpa, diffPressureHpa;
    SensorModelISA::EstimateAtmosphere(gpsPosition, linVelNed,
                                       temperatureKelvin, absPressureHpa, diffPressureHpa);

    // Publish state to communicator
    _sensors.attitudeSensor.publish(attitudeFrdToNed);
    _sensors.imuSensor.publish(accFrd, gyroFrd);
    _sensors.velocitySensor_.publish(linVelNed, angVelFrd);
    _sensors.magSensor.publish(gpsPosition, attitudeFrdToNed);
    _sensors.rawAirDataSensor.publish(absPressureHpa, diffPressureHpa, temperatureKelvin);
    _sensors.pressureSensor.publish(absPressureHpa);
    _sensors.temperatureSensor.publish(temperatureKelvin);
    _sensors.gpsSensor.publish(gpsPosition, linVelNed);

    std::vector<double> motorsRpm;
    if(uavDynamicsSim_->getMotorsRpm(motorsRpm)){
        _sensors.escStatusSensor.publish(motorsRpm);
        if(motorsRpm.size() == 5){
            _sensors.iceStatusSensor.publish(motorsRpm[4]);
        }
    }

    ///< @todo Simplified Fuel tank model, refactor it
    static double fuelLevelPercentage = 100.0;
    if(motorsRpm.size() == 5 && motorsRpm[4] >= 1) {
        fuelLevelPercentage -= 0.002;
        if(fuelLevelPercentage < 0) {
            fuelLevelPercentage = 0;
        }
    }
    _sensors.fuelTankSensor.publish(fuelLevelPercentage);

    ///< @todo Battery is just constant, add model
    _sensors.batteryInfoSensor.publish(90.0);
}

void Uav_Dynamics::publishToRos(double period){
    while(ros::ok()){
        auto crnt_time = std::chrono::system_clock::now();
        auto sleed_period = std::chrono::microseconds(int(1000000 * period * clockScale_));
        auto time_point = crnt_time + sleed_period;
        rosPubCounter_++;

        publishState();

        static auto next_time = std::chrono::system_clock::now();
        if(crnt_time > next_time){
            publishMarkers();
            next_time += std::chrono::milliseconds(int(50));
        }

        std::this_thread::sleep_until(time_point);
    }
}

void Uav_Dynamics::actuatorsCallback(sensor_msgs::Joy::Ptr msg){
    prevActuatorsTimestampUsec_ = lastActuatorsTimestampUsec_;
    lastActuatorsTimestampUsec_ = msg->header.stamp.toNSec() / 1000;
    auto crntDelayUsec = lastActuatorsTimestampUsec_ - prevActuatorsTimestampUsec_;
    if(crntDelayUsec > maxDelayUsec_){
        maxDelayUsec_ = crntDelayUsec;
    }
    actuatorsMsgCounter_++;

    for(size_t idx = 0; idx < msg->axes.size(); idx++){
        actuators_[idx] = msg->axes[idx];
    }

    if (_scenarioType == 1) {
        actuators_[7] = 0.0;
    }
}

void Uav_Dynamics::armCallback(std_msgs::Bool msg){
    if(armed_ != msg.data){
        /**
         * @note why it publish few times when sim starts? hack: use throttle
         */
        ROS_INFO_STREAM_THROTTLE(1, "cmd: " << (msg.data ? "Arm" : "Disarm"));
    }
    armed_ = msg.data;
}

void Uav_Dynamics::scenarioCallback(std_msgs::UInt8 msg){
    _scenarioType = msg.data;
    _sensors.iceStatusSensor.start_stall_emulation();
}

void Uav_Dynamics::calibrationCallback(std_msgs::UInt8 msg){
    if(calibrationType_ != msg.data){
        ROS_INFO_STREAM_THROTTLE(1, "calibration type: " << msg.data + 0);
    }
    calibrationType_ = static_cast<UavDynamicsSimBase::CalibrationType_t>(msg.data);
}

/**
 * @brief Perform TF transform between GLOBAL_FRAME -> UAV_FRAME in ROS (enu/flu) format
 */
void Uav_Dynamics::publishState(void){
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = GLOBAL_FRAME_ID;

    auto position = uavDynamicsSim_->getVehiclePosition();
    auto attitude = uavDynamicsSim_->getVehicleAttitude();
    Eigen::Vector3d enuPosition;
    Eigen::Quaterniond fluAttitude;
    if(dynamicsNotation_ == PX4_NED_FRD){
        enuPosition = Converter::nedToEnu(position);
        fluAttitude = Converter::frdNedTofluEnu(attitude);
    }else{
        enuPosition = position;
        fluAttitude = attitude;
    }

    transform.transform.translation.x = enuPosition[0];
    transform.transform.translation.y = enuPosition[1];
    transform.transform.translation.z = enuPosition[2];
    transform.transform.rotation.x = fluAttitude.x();
    transform.transform.rotation.y = fluAttitude.y();
    transform.transform.rotation.z = fluAttitude.z();
    transform.transform.rotation.w = fluAttitude.w();
    transform.child_frame_id = UAV_FRAME_ID;
    tfPub_.sendTransform(transform);

    transform.transform.rotation.x = 0;
    transform.transform.rotation.y = 0;
    transform.transform.rotation.z = 0;
    transform.transform.rotation.w = 1;
    transform.child_frame_id = UAV_FIXED_FRAME_ID;
    tfPub_.sendTransform(transform);
}

visualization_msgs::Marker& Uav_Dynamics::makeArrow(const Eigen::Vector3d& vector3D,
                                                    const Eigen::Vector3d& rgbColor,
                                                    const char* frameId = UAV_FRAME_ID){
    auto fluVector = Converter::frdToFlu(vector3D);
    arrowMarkers_.header.frame_id = frameId;
    arrowMarkers_.points[1].x = fluVector[0];
    arrowMarkers_.points[1].y = fluVector[1];
    arrowMarkers_.points[1].z = fluVector[2];
    arrowMarkers_.color.r = rgbColor[0];
    arrowMarkers_.color.g = rgbColor[1];
    arrowMarkers_.color.b = rgbColor[2];
    return arrowMarkers_;
}

void Uav_Dynamics::initMarkers(){
    arrowMarkers_.id = 0;
    arrowMarkers_.type = visualization_msgs::Marker::ARROW;
    arrowMarkers_.action = visualization_msgs::Marker::ADD;
    arrowMarkers_.pose.orientation.w = 1;
    arrowMarkers_.scale.x = 0.05;   // radius of cylinder
    arrowMarkers_.scale.y = 0.1;
    arrowMarkers_.scale.z = 0.03;   // scale of hat
    arrowMarkers_.lifetime = ros::Duration();
    arrowMarkers_.color.a = 1.0;

    geometry_msgs::Point startPoint, endPoint;
    startPoint.x = 0;
    startPoint.y = 0;
    startPoint.z = 0;
    endPoint.x = 0;
    endPoint.y = 0;
    endPoint.z = 0;

    arrowMarkers_.points.push_back(startPoint);
    arrowMarkers_.points.push_back(endPoint);
}

/**
 * @brief Publish forces and moments of vehicle
 */
void Uav_Dynamics::publishMarkers(void){
    if(dynamicsType_ == DYNAMICS_INNO_VTOL){
        arrowMarkers_.header.stamp = ros::Time();
        Eigen::Vector3d MOMENT_COLOR(0.5, 0.5, 0.0),
                        MOTORS_FORCES_COLOR(0.0, 0.5, 0.5),
                        SPEED_COLOR(0.7, 0.5, 1.3),
                        LIFT_FORCE(0.8, 0.2, 0.3),
                        DRAG_FORCE(0.2, 0.8, 0.3),
                        SIDE_FORCE(0.2, 0.3, 0.8);

        // publish moments
        auto Maero = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getMaero();
        aeroMomentPub_.publish(makeArrow(Maero, MOMENT_COLOR));

        auto Mmotors = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getMmotors();
        for(size_t motorIdx = 0; motorIdx < 5; motorIdx++){
            motorsMomentsPub_[motorIdx].publish(makeArrow(Mmotors[motorIdx],
                                                          MOMENT_COLOR,
                                                          MOTOR_NAMES[motorIdx].c_str()));
        }

        auto Mtotal = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getMtotal();
        totalMomentPub_.publish(makeArrow(Mtotal, MOMENT_COLOR));

        auto Msteer = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getMsteer();
        controlSurfacesMomentPub_.publish(makeArrow(Msteer, MOMENT_COLOR));

        auto Mairspeed = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getMairspeed();
        aoaMomentPub_.publish(makeArrow(Mairspeed, MOMENT_COLOR));


        // publish forces
        auto Faero = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getFaero();
        aeroForcePub_.publish(makeArrow(Faero / 10, MOTORS_FORCES_COLOR));

        auto Fmotors = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getFmotors();
        for(size_t motorIdx = 0; motorIdx < 5; motorIdx++){
            motorsForcesPub_[motorIdx].publish(makeArrow(Fmotors[motorIdx] / 10,
                                                         MOTORS_FORCES_COLOR,
                                                         MOTOR_NAMES[motorIdx].c_str()));
        }

        auto Ftotal = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getFtotal();
        totalForcePub_.publish(makeArrow(Ftotal, Eigen::Vector3d(0.0, 1.0, 1.0)));

        auto velocity = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getBodyLinearVelocity();
        velocityPub_.publish(makeArrow(velocity, SPEED_COLOR));

        auto Flift = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getFlift();
        liftForcePub_.publish(makeArrow(Flift / 10, LIFT_FORCE));

        auto Fdrug = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getFdrug();
        drugForcePub_.publish(makeArrow(Fdrug / 10, DRAG_FORCE));

        auto Fside = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim_)->getFside();
        sideForcePub_.publish(makeArrow(Fside / 10, SIDE_FORCE));
    }
}

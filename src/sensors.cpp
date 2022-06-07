/**
 * @file sensors.cpp
 * @author Dmitry Ponomarev
 */

#include "sensors.hpp"
#include "cs_converter.hpp"

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>

///< uavcan_msgs are deprecated and should be removed asap
#include <uavcan_msgs/RawAirData.h>
#include <uavcan_msgs/StaticPressure.h>
#include <uavcan_msgs/StaticTemperature.h>
#include <uavcan_msgs/Fix.h>
#include <uavcan_msgs/EscStatus.h>
#include <uavcan_msgs/IceReciprocatingStatus.h>
#include <uavcan_msgs/IceFuelTankStatus.h>


static const double MAG_NOISE = 0.0002;
static const double STATIC_PRESSURE_NOISE = 0.1;
static const double DIFF_PRESSURE_NOISE_PA = 5;
static const double TEMPERATURE_NOISE = 0.1;


AttitudeSensor::AttitudeSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<geometry_msgs::QuaternionStamped>(topic, 5);
}
bool AttitudeSensor::publish(const Eigen::Quaterniond& attitudeFrdToNed) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!isEnabled_ || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    geometry_msgs::QuaternionStamped msg;
    msg.quaternion.x = attitudeFrdToNed.x();
    msg.quaternion.y = attitudeFrdToNed.y();
    msg.quaternion.z = attitudeFrdToNed.z();
    msg.quaternion.w = attitudeFrdToNed.w();
    msg.header.stamp = ros::Time::now();

    publisher_.publish(msg);
    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

VelocitySensor::VelocitySensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<geometry_msgs::Twist>(topic, 5);
}
bool VelocitySensor::publish(const Eigen::Vector3d& linVelNed, const Eigen::Vector3d& angVelFrd) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!isEnabled_ || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = linVelNed[0];
    msg.linear.y = linVelNed[1];
    msg.linear.z = linVelNed[2];
    msg.angular.x = angVelFrd[0];
    msg.angular.y = angVelFrd[1];
    msg.angular.z = angVelFrd[2];

    publisher_.publish(msg);
    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

ImuSensor::ImuSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<sensor_msgs::Imu>(topic, 5);
}
bool ImuSensor::publish(const Eigen::Vector3d& accFrd, const Eigen::Vector3d& gyroFrd) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!isEnabled_ || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.angular_velocity.x = gyroFrd[0];
    msg.angular_velocity.y = gyroFrd[1];
    msg.angular_velocity.z = gyroFrd[2];
    msg.linear_acceleration.x = accFrd[0];
    msg.linear_acceleration.y = accFrd[1];
    msg.linear_acceleration.z = accFrd[2];

    publisher_.publish(msg);
    if (nextPubTimeSec_ + PERIOD > crntTimeSec) {
        nextPubTimeSec_ += PERIOD;
    } else {
        nextPubTimeSec_ = crntTimeSec + PERIOD;
    }
    return true;
}

MagSensor::MagSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<sensor_msgs::MagneticField>(topic, 5);
}
bool MagSensor::publish(const Eigen::Vector3d& geoPosition, const Eigen::Quaterniond& attitudeFrdToNed) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!isEnabled_ || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    sensor_msgs::MagneticField msg;
    Eigen::Vector3d magEnu;
    geographiclib_conversions::MagneticField(
        geoPosition.x(), geoPosition.y(), geoPosition.z(),
        magEnu.x(), magEnu.y(), magEnu.z());
    Eigen::Vector3d magFrd = attitudeFrdToNed.inverse() * Converter::enuToNed(magEnu);
    msg.header.stamp = ros::Time();
    msg.magnetic_field.x = magFrd[0] + MAG_NOISE * normalDistribution_(randomGenerator_);
    msg.magnetic_field.y = magFrd[1] + MAG_NOISE * normalDistribution_(randomGenerator_);
    msg.magnetic_field.z = magFrd[2] + MAG_NOISE * normalDistribution_(randomGenerator_);

    publisher_.publish(msg);
    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

RawAirDataSensor::RawAirDataSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::RawAirData>(topic, 5);
}
bool RawAirDataSensor::publish(float absPressureHpa, float diffPressure, float staticTemperature) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!isEnabled_ || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    uavcan_msgs::RawAirData msg;
    msg.header.stamp = ros::Time();
    msg.static_pressure = absPressureHpa * 100;
    msg.differential_pressure = diffPressure * 100;
    msg.static_air_temperature = staticTemperature;
    msg.static_pressure += STATIC_PRESSURE_NOISE * normalDistribution_(randomGenerator_);
    msg.differential_pressure += DIFF_PRESSURE_NOISE_PA * normalDistribution_(randomGenerator_);
    msg.static_air_temperature += TEMPERATURE_NOISE * normalDistribution_(randomGenerator_);

    publisher_.publish(msg);
    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

StaticPressureSensor::StaticPressureSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    old_publisher_ = node_handler_->advertise<uavcan_msgs::StaticPressure>(topic, 5);
    publisher_ = node_handler_->advertise<std_msgs::Float32>("/uav/baro_pressure", 5);
}
bool StaticPressureSensor::publish(float staticPressureHpa) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!isEnabled_ || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    uavcan_msgs::StaticPressure old_msg;
    old_msg.header.stamp = ros::Time();
    old_msg.static_pressure = staticPressureHpa * 100;
    old_msg.static_pressure += STATIC_PRESSURE_NOISE * normalDistribution_(randomGenerator_);
    old_publisher_.publish(old_msg);

    std_msgs::Float32 msg;
    msg.data = staticPressureHpa * 100;
    msg.data += STATIC_PRESSURE_NOISE * normalDistribution_(randomGenerator_);
    publisher_.publish(msg);

    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

StaticTemperatureSensor::StaticTemperatureSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    old_publisher_ = node_handler_->advertise<uavcan_msgs::StaticTemperature>(topic, 5);
    publisher_ = node_handler_->advertise<std_msgs::Float32>("/uav/baro_temperature", 5);
}
bool StaticTemperatureSensor::publish(float staticTemperature) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!isEnabled_ || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    uavcan_msgs::StaticTemperature old_msg;
    old_msg.header.stamp = ros::Time();
    old_msg.static_temperature = staticTemperature + 5;
    old_msg.static_temperature += TEMPERATURE_NOISE * normalDistribution_(randomGenerator_);
    old_publisher_.publish(old_msg);

    std_msgs::Float32 msg;
    msg.data = staticTemperature + 5;
    msg.data += TEMPERATURE_NOISE * normalDistribution_(randomGenerator_);
    publisher_.publish(msg);

    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

EscStatusSensor::EscStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::EscStatus>(topic, 10);
}
bool EscStatusSensor::publish(const std::vector<double>& rpm) {
    ///< The idea here is to publish each esc status with equal interval instead of burst
    auto crntTimeSec = ros::Time::now().toSec();
    if(isEnabled_ && rpm.size() > 0 && rpm.size() <= 8 && (nextPubTimeSec_ < crntTimeSec)){
        uavcan_msgs::EscStatus escStatusMsg;
        if(nextEscIdx_ >= rpm.size()){
            nextEscIdx_ = 0;
        }
        escStatusMsg.esc_index = nextEscIdx_;
        escStatusMsg.rpm = rpm[nextEscIdx_];
        publisher_.publish(escStatusMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD / rpm.size();
        nextEscIdx_++;
    }
    return true;
}

GpsSensor::GpsSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::Fix>(topic, 5);
    position_publisher_ = node_handler_->advertise<sensor_msgs::NavSatFix>("/uav/gps_point", 5);
}
bool GpsSensor::publish(const Eigen::Vector3d& gpsPosition, const Eigen::Vector3d& nedVelocity) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(!isEnabled_ || (nextPubTimeSec_ > crntTimeSec)){
        return false;
    }

    uavcan_msgs::Fix fixMsg;
    fixMsg.header.stamp = ros::Time::now();
    fixMsg.latitude_deg_1e8 = gpsPosition[0] * 1e+8;
    fixMsg.longitude_deg_1e8 = gpsPosition[1] * 1e+8;
    fixMsg.height_msl_mm = gpsPosition[2] * 1e+3;
    fixMsg.ned_velocity.x = nedVelocity[0];
    fixMsg.ned_velocity.y = nedVelocity[1];
    fixMsg.ned_velocity.z = nedVelocity[2];
    fixMsg.sats_used = 10;
    fixMsg.status = 3;
    fixMsg.pdop = 1;
    publisher_.publish(fixMsg);

    sensor_msgs::NavSatFix gps_position_msg;
    gps_position_msg.latitude = gpsPosition[0] * 1e+8;
    gps_position_msg.longitude = gpsPosition[1] * 1e+8;
    gps_position_msg.altitude = gpsPosition[2] * 1e+3;
    position_publisher_.publish(gps_position_msg);

    nextPubTimeSec_ = crntTimeSec + PERIOD;
    return true;
}

IceStatusSensor::IceStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::IceReciprocatingStatus>(topic, 16);
}
bool IceStatusSensor::publish(double rpm) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(isEnabled_ && (nextPubTimeSec_ < crntTimeSec)){
        uavcan_msgs::IceReciprocatingStatus iceStatusMsg;
        iceStatusMsg.engine_speed_rpm = rpm;
        publisher_.publish(iceStatusMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD;
    }
    return true;
}

FuelTankStatusSensor::FuelTankStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<uavcan_msgs::IceFuelTankStatus>(topic, 16);
}
bool FuelTankStatusSensor::publish(double fuelLevelPercentage) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(isEnabled_ && (nextPubTimeSec_ < crntTimeSec)){
        uavcan_msgs::IceFuelTankStatus fuelTankMsg;
        fuelTankMsg.available_fuel_volume_percent = fuelLevelPercentage;
        publisher_.publish(fuelTankMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD;
    }
    return true;
}


BatteryInfoStatusSensor::BatteryInfoStatusSensor(ros::NodeHandle* nh, const char* topic, double period) : BaseSensor(nh, period){
    publisher_ = node_handler_->advertise<sensor_msgs::BatteryState>(topic, 16);
}
bool BatteryInfoStatusSensor::publish(double percentage) {
    auto crntTimeSec = ros::Time::now().toSec();
    if(isEnabled_ && (nextPubTimeSec_ < crntTimeSec)){
        sensor_msgs::BatteryState batteryInfoMsg;
        batteryInfoMsg.voltage = 4.1;
        batteryInfoMsg.percentage = percentage;
        batteryInfoMsg.capacity = 6;
        publisher_.publish(batteryInfoMsg);
        nextPubTimeSec_ = crntTimeSec + PERIOD;
    }
    return true;
}

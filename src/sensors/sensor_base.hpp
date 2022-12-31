/* 
 * Copyright (c) 2020-2022 RaccoonLab.
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

#ifndef SENSORS_SENSOR_BASE_HPP
#define SENSORS_SENSOR_BASE_HPP

#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <Eigen/Geometry>
#include <random>
#include <geographiclib_conversions/geodetic_conv.hpp>

class BaseSensor{
    public:
        BaseSensor() = delete;
        BaseSensor(ros::NodeHandle* nh, double period): node_handler_(nh), PERIOD(period) {};
        void enable() {_isEnabled = true;}
        void disable() {_isEnabled = false;}
    protected:
        ros::NodeHandle* node_handler_;
        bool _isEnabled{false};
        const double PERIOD;
        ros::Publisher publisher_;
        double nextPubTimeSec_ = 0;

        std::default_random_engine randomGenerator_;
        std::normal_distribution<double> normalDistribution_{std::normal_distribution<double>(0.0, 1.0)};
};

class AttitudeSensor : public BaseSensor{
    public:
        AttitudeSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(const Eigen::Quaterniond& attitudeFrdToNed);
};

class BatteryInfoSensor : public BaseSensor{
    public:
        BatteryInfoSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(double rpm);
};

class EscStatusSensor : public BaseSensor{
    public:
        EscStatusSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(const std::vector<double>& rpm);
    private:
        uint8_t nextEscIdx_ = 0;
};

class FuelTankSensor : public BaseSensor{
    public:
        FuelTankSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(double rpm);
};

class GpsSensor : public BaseSensor{
    public:
        GpsSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(const Eigen::Vector3d& gpsPosition, const Eigen::Vector3d& nedVelocity);
    private:
        ros::Publisher yaw_publisher_;
        ros::Publisher position_publisher_;
        ros::Publisher velocity_publisher_;
};

class ImuSensor : public BaseSensor{
    public:
        ImuSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(const Eigen::Vector3d& accFrd, const Eigen::Vector3d& gyroFrd);
};

class MagSensor : public BaseSensor{
    public:
        MagSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(const Eigen::Vector3d& geoPosition, const Eigen::Quaterniond& attitudeFrdToNed);
};

class RawAirDataSensor : public BaseSensor{
    public:
        RawAirDataSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(float staticPressureHpa, float diffPressureHpa, float staticTemperature);
    private:
        ros::Publisher _old_publisher;
};

class PressureSensor : public BaseSensor{
    public:
        PressureSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(float staticPressureHpa);
};

class TemperatureSensor : public BaseSensor{
    public:
        TemperatureSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(float staticTemperature);
};

class VelocitySensor : public BaseSensor{
    public:
        VelocitySensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(const Eigen::Vector3d& linVelNed, const Eigen::Vector3d& angVelFrd);
};


#endif  // SENSORS_SENSOR_BASE_HPP

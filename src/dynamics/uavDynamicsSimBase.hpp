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
 * Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
 */


#ifndef UAV_DYNAMICS_SIM_BASE_HPP
#define UAV_DYNAMICS_SIM_BASE_HPP

#include <Eigen/Geometry>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>


class UavDynamicsSimBase{
public:
    UavDynamicsSimBase() = default;
    virtual ~UavDynamicsSimBase() = default;

    /**
     * @brief Use rosparam here to initialize sim 
     * @return -1 if error occures and simulation can't start
     */
    virtual int8_t init() = 0;
    virtual void setInitialPosition(const Eigen::Vector3d & position,
                                    const Eigen::Quaterniond& attitude) = 0;
    virtual void setWindParameter(Eigen::Vector3d windMeanVelocityNED, double wind_velocityVariance) {}

    virtual void land() {
        // do nothing by default
    }
    virtual void process(double dt_secs, const std::vector<double>& setpoint) = 0;

    virtual Eigen::Vector3d getVehiclePosition() const = 0;
    virtual Eigen::Quaterniond getVehicleAttitude() const = 0;
    virtual Eigen::Vector3d getVehicleVelocity(void) const = 0;
    virtual Eigen::Vector3d getVehicleAirspeed() const = 0;
    virtual Eigen::Vector3d getVehicleAngularVelocity(void) const = 0;
    virtual void getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput) = 0;
    virtual bool getMotorsRpm(std::vector<double>& motorsRpm);

    enum class SimMode_t{
        NORMAL = 0,

        MAG_1_NORMAL = 1,           // ROLL OK              ROTATE YAW POSITIVE
        MAG_2_OVERTURNED = 2,       // ROLL INVERTED        ROTATE YAW NEGATIVE
        MAG_3_HEAD_DOWN = 3,        // PITCH POSITIVE pi/2  ROTATE YAW POSITIVE
        MAG_4_HEAD_UP = 4,          // PITCH NEGATIVE pi/2  ROTATE YAW NEGATIVE
        MAG_5_TURNED_LEFT = 5,      // ROLL POSITIVE pi/2   ROTATE YAW POSITIVE
        MAG_6_TURNED_RIGHT = 6,     // ROLL NEGATIVE pi/2   ROTATE YAW NEGATIVE
        MAG_7_ARDUPILOT = 7,        // Random rotations
        MAG_8_ARDUPILOT = 8,        // Random rotations
        MAG_9_ARDUPILOT = 9,        // Random rotations

        ACC_1_NORMAL = 11,          // ROLL OK
        ACC_2_OVERTURNED = 12,      // ROLL INVERTED
        ACC_3_HEAD_DOWN = 13,       // PITCH POSITIVE pi/2
        ACC_4_HEAD_UP = 14,         // PITCH NEGATIVE pi/2
        ACC_5_TURNED_LEFT = 15,     // ROLL POSITIVE pi/2
        ACC_6_TURNED_RIGHT = 16,    // ROLL NEGATIVE pi/2

        AIRSPEED = 21,              // Emulate airspeed
    };
    virtual int8_t calibrate(SimMode_t calibrationType) { return -1; }
};


#endif  // UAV_DYNAMICS_SIM_BASE_HPP

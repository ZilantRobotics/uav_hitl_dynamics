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

#include "rviz_visualization.hpp"
#include "cs_converter.hpp"
#include "vtolDynamicsSim.hpp"

static const constexpr uint8_t PX4_NED_FRD = 0;
static const constexpr uint8_t ROS_ENU_FLU = 1;

static const std::array<std::string, 5> MOTOR_NAMES = {"motor0", "motor1", "motor2", "motor3", "ICE"};
static const char GLOBAL_FRAME_ID[] = "world";
static const char UAV_FRAME_ID[] = "uav/enu";
static const char UAV_FIXED_FRAME_ID[] = "uav/com";

RvizVisualizator::RvizVisualizator(ros::NodeHandle& nh) : node(nh) {
}

int8_t RvizVisualizator::init(const std::shared_ptr<UavDynamicsSimBase>& uavDynamicsSim_) {
    uavDynamicsSim = uavDynamicsSim_;

    if (uavDynamicsSim_ == nullptr) {
        return -1;
    }

    initMarkers();
    totalForcePub = node.advertise<visualization_msgs::Marker>("/uav/Ftotal", 1);
    aeroForcePub = node.advertise<visualization_msgs::Marker>("/uav/Faero", 1);
    motorsForcesPub[0] = node.advertise<visualization_msgs::Marker>("/uav/Fmotor0", 1);
    motorsForcesPub[1] = node.advertise<visualization_msgs::Marker>("/uav/Fmotor1", 1);
    motorsForcesPub[2] = node.advertise<visualization_msgs::Marker>("/uav/Fmotor2", 1);
    motorsForcesPub[3] = node.advertise<visualization_msgs::Marker>("/uav/Fmotor3", 1);
    motorsForcesPub[4] = node.advertise<visualization_msgs::Marker>("/uav/Fmotor4", 1);
    liftForcePub = node.advertise<visualization_msgs::Marker>("/uav/liftForce", 1);
    drugForcePub = node.advertise<visualization_msgs::Marker>("/uav/drugForce", 1);
    sideForcePub = node.advertise<visualization_msgs::Marker>("/uav/sideForce", 1);

    totalMomentPub = node.advertise<visualization_msgs::Marker>("/uav/Mtotal", 1);
    aeroMomentPub = node.advertise<visualization_msgs::Marker>("/uav/Maero", 1);
    controlSurfacesMomentPub = node.advertise<visualization_msgs::Marker>("/uav/McontrolSurfaces", 1);
    aoaMomentPub = node.advertise<visualization_msgs::Marker>("/uav/Maoa", 1);
    motorsMomentsPub[0] = node.advertise<visualization_msgs::Marker>("/uav/Mmotor0", 1);
    motorsMomentsPub[1] = node.advertise<visualization_msgs::Marker>("/uav/Mmotor1", 1);
    motorsMomentsPub[2] = node.advertise<visualization_msgs::Marker>("/uav/Mmotor2", 1);
    motorsMomentsPub[3] = node.advertise<visualization_msgs::Marker>("/uav/Mmotor3", 1);
    motorsMomentsPub[4] = node.advertise<visualization_msgs::Marker>("/uav/Mmotor4", 1);

    velocityPub = node.advertise<visualization_msgs::Marker>("/uav/linearVelocity", 1);

    return 0;
}


void RvizVisualizator::publish(uint8_t) {
    arrowMarkers.header.stamp = ros::Time();
    const Eigen::Vector3d MOMENT_COLOR(0.5, 0.5, 0.0);
    const Eigen::Vector3d MOTORS_FORCES_COLOR(0.0, 0.5, 0.5);
    const Eigen::Vector3d SPEED_COLOR(0.7, 0.5, 1.3);
    const Eigen::Vector3d LIFT_FORCE(0.8, 0.2, 0.3);
    const Eigen::Vector3d DRAG_FORCE(0.2, 0.8, 0.3);
    const Eigen::Vector3d SIDE_FORCE(0.2, 0.3, 0.8);

    auto moments = dynamic_cast<VtolDynamics*>(uavDynamicsSim.get())->getMoments();
    auto forces = dynamic_cast<VtolDynamics*>(uavDynamicsSim.get())->getForces();

    // publish moments
    aeroMomentPub.publish(makeArrow(moments.aero, MOMENT_COLOR, UAV_FRAME_ID));

    for(size_t motorIdx = 0; motorIdx < 5; motorIdx++){
        motorsMomentsPub[motorIdx].publish(makeArrow(moments.motors[motorIdx],
                                                        MOMENT_COLOR,
                                                        MOTOR_NAMES[motorIdx].c_str()));
    }

    totalMomentPub.publish(makeArrow(moments.total, MOMENT_COLOR, UAV_FRAME_ID));

    controlSurfacesMomentPub.publish(makeArrow(moments.steer, MOMENT_COLOR, UAV_FRAME_ID));

    aoaMomentPub.publish(makeArrow(moments.airspeed, MOMENT_COLOR, UAV_FRAME_ID));



    // publish forces
    aeroForcePub.publish(makeArrow(forces.aero / 10, MOTORS_FORCES_COLOR, UAV_FRAME_ID));

    for(size_t motorIdx = 0; motorIdx < 5; motorIdx++){
        motorsForcesPub[motorIdx].publish(makeArrow(forces.motors[motorIdx] / 10,
                                                        MOTORS_FORCES_COLOR,
                                                        MOTOR_NAMES[motorIdx].c_str()));
    }

    totalForcePub.publish(makeArrow(forces.total, Eigen::Vector3d(0.0, 1.0, 1.0), UAV_FRAME_ID));

    auto velocity = dynamic_cast<VtolDynamics*>(uavDynamicsSim.get())->getBodyLinearVelocity();
    velocityPub.publish(makeArrow(velocity, SPEED_COLOR, UAV_FRAME_ID));

    liftForcePub.publish(makeArrow(forces.lift / 10, LIFT_FORCE, UAV_FRAME_ID));
    drugForcePub.publish(makeArrow(forces.drug / 10, DRAG_FORCE, UAV_FRAME_ID));
    sideForcePub.publish(makeArrow(forces.side / 10, SIDE_FORCE, UAV_FRAME_ID));
}

visualization_msgs::Marker& RvizVisualizator::makeArrow(const Eigen::Vector3d& vector3D,
                                                        const Eigen::Vector3d& rgbColor,
                                                        const char*){
    auto fluVector = Converter::frdToFlu(vector3D);
    arrowMarkers.header.frame_id = UAV_FRAME_ID;
    arrowMarkers.points[1].x = fluVector[0];
    arrowMarkers.points[1].y = fluVector[1];
    arrowMarkers.points[1].z = fluVector[2];
    arrowMarkers.color.r = static_cast<float>(rgbColor[0]);
    arrowMarkers.color.g = static_cast<float>(rgbColor[1]);
    arrowMarkers.color.b = static_cast<float>(rgbColor[2]);
    return arrowMarkers;
}

void RvizVisualizator::initMarkers(){
    arrowMarkers.id = 0;
    arrowMarkers.type = visualization_msgs::Marker::ARROW;
    arrowMarkers.action = visualization_msgs::Marker::ADD;
    arrowMarkers.pose.orientation.w = 1;
    arrowMarkers.scale.x = 0.05;   // radius of cylinder
    arrowMarkers.scale.y = 0.1;
    arrowMarkers.scale.z = 0.03;   // scale of hat
    arrowMarkers.lifetime = ros::Duration();
    arrowMarkers.color.a = 1.0;

    geometry_msgs::Point startPoint;
    startPoint.x = 0;
    startPoint.y = 0;
    startPoint.z = 0;
    arrowMarkers.points.push_back(startPoint);

    geometry_msgs::Point endPoint;
    endPoint.x = 0;
    endPoint.y = 0;
    endPoint.z = 0;
    arrowMarkers.points.push_back(endPoint);
}

/**
 * @brief Perform TF transform between GLOBAL_FRAME -> UAV_FRAME in ROS (enu/flu) format
 */
void RvizVisualizator::publishTf(uint8_t dynamicsNotation){
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = GLOBAL_FRAME_ID;

    auto position = uavDynamicsSim->getVehiclePosition();
    auto attitude = uavDynamicsSim->getVehicleAttitude();
    Eigen::Vector3d enuPosition;
    Eigen::Quaterniond fluAttitude;
    if(dynamicsNotation == PX4_NED_FRD){
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
    tfPub.sendTransform(transform);

    transform.transform.rotation.x = 0;
    transform.transform.rotation.y = 0;
    transform.transform.rotation.z = 0;
    transform.transform.rotation.w = 1;
    transform.child_frame_id = UAV_FIXED_FRAME_ID;
    tfPub.sendTransform(transform);
}

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

// @todo fix this hack
#define PX4_NED_FRD 0
#define ROS_ENU_FLU 1

const std::string MOTOR_NAMES[5] = {"motor0", "motor1", "motor2", "motor3", "ICE"};
static char GLOBAL_FRAME_ID[] = "world";
static char UAV_FRAME_ID[] = "uav/enu";
static char UAV_FIXED_FRAME_ID[] = "uav/com";

RvizVisualizator::RvizVisualizator(ros::NodeHandle& nh) : node(nh) {
}

int8_t RvizVisualizator::init(UavDynamicsSimBase* uavDynamicsSim_) {
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


void RvizVisualizator::publish(uint8_t dynamicsNotation) {
    arrowMarkers.header.stamp = ros::Time();
    Eigen::Vector3d MOMENT_COLOR(0.5, 0.5, 0.0),
                    MOTORS_FORCES_COLOR(0.0, 0.5, 0.5),
                    SPEED_COLOR(0.7, 0.5, 1.3),
                    LIFT_FORCE(0.8, 0.2, 0.3),
                    DRAG_FORCE(0.2, 0.8, 0.3),
                    SIDE_FORCE(0.2, 0.3, 0.8);

    // publish moments
    auto Maero = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getMaero();
    aeroMomentPub.publish(makeArrow(Maero, MOMENT_COLOR, UAV_FRAME_ID));

    auto Mmotors = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getMmotors();
    for(size_t motorIdx = 0; motorIdx < 5; motorIdx++){
        motorsMomentsPub[motorIdx].publish(makeArrow(Mmotors[motorIdx],
                                                        MOMENT_COLOR,
                                                        MOTOR_NAMES[motorIdx].c_str()));
    }

    auto Mtotal = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getMtotal();
    totalMomentPub.publish(makeArrow(Mtotal, MOMENT_COLOR, UAV_FRAME_ID));

    auto Msteer = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getMsteer();
    controlSurfacesMomentPub.publish(makeArrow(Msteer, MOMENT_COLOR, UAV_FRAME_ID));

    auto Mairspeed = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getMairspeed();
    aoaMomentPub.publish(makeArrow(Mairspeed, MOMENT_COLOR, UAV_FRAME_ID));


    // publish forces
    auto Faero = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getFaero();
    aeroForcePub.publish(makeArrow(Faero / 10, MOTORS_FORCES_COLOR, UAV_FRAME_ID));

    auto Fmotors = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getFmotors();
    for(size_t motorIdx = 0; motorIdx < 5; motorIdx++){
        motorsForcesPub[motorIdx].publish(makeArrow(Fmotors[motorIdx] / 10,
                                                        MOTORS_FORCES_COLOR,
                                                        MOTOR_NAMES[motorIdx].c_str()));
    }

    auto Ftotal = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getFtotal();
    totalForcePub.publish(makeArrow(Ftotal, Eigen::Vector3d(0.0, 1.0, 1.0), UAV_FRAME_ID));

    auto velocity = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getBodyLinearVelocity();
    velocityPub.publish(makeArrow(velocity, SPEED_COLOR, UAV_FRAME_ID));

    auto Flift = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getFlift();
    liftForcePub.publish(makeArrow(Flift / 10, LIFT_FORCE, UAV_FRAME_ID));

    auto Fdrug = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getFdrug();
    drugForcePub.publish(makeArrow(Fdrug / 10, DRAG_FORCE, UAV_FRAME_ID));

    auto Fside = static_cast<InnoVtolDynamicsSim*>(uavDynamicsSim)->getFside();
    sideForcePub.publish(makeArrow(Fside / 10, SIDE_FORCE, UAV_FRAME_ID));
}

visualization_msgs::Marker& RvizVisualizator::makeArrow(const Eigen::Vector3d& vector3D,
                                                        const Eigen::Vector3d& rgbColor,
                                                        const char* frameId = UAV_FRAME_ID){
    auto fluVector = Converter::frdToFlu(vector3D);
    arrowMarkers.header.frame_id = UAV_FRAME_ID;
    arrowMarkers.points[1].x = fluVector[0];
    arrowMarkers.points[1].y = fluVector[1];
    arrowMarkers.points[1].z = fluVector[2];
    arrowMarkers.color.r = rgbColor[0];
    arrowMarkers.color.g = rgbColor[1];
    arrowMarkers.color.b = rgbColor[2];
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

    geometry_msgs::Point startPoint, endPoint;
    startPoint.x = 0;
    startPoint.y = 0;
    startPoint.z = 0;
    endPoint.x = 0;
    endPoint.y = 0;
    endPoint.z = 0;

    arrowMarkers.points.push_back(startPoint);
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

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

#ifndef SRC_RVIZ_VISUALIZATION_HPP_
#define SRC_RVIZ_VISUALIZATION_HPP_

#include <array>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include "uavDynamicsSimBase.hpp"

class RvizVisualizator {
public:
    explicit RvizVisualizator(ros::NodeHandle& nh);
    int8_t init(const std::shared_ptr<UavDynamicsSimBase>& uavDynamicsSim_);

    // common
    void publishTf(uint8_t dynamicsNotation);

    // only for vtol
    void publish(uint8_t dynamicsNotation);

private:
    void initMarkers();
    visualization_msgs::Marker& makeArrow(const Eigen::Vector3d& vector3D,
                                          const Eigen::Vector3d& rgbColor,
                                          const char* frameId);

    ros::NodeHandle& node;
    std::shared_ptr<UavDynamicsSimBase> uavDynamicsSim;

    visualization_msgs::Marker arrowMarkers;

    ros::Publisher totalForcePub;
    ros::Publisher aeroForcePub;
    std::array<ros::Publisher, 5> motorsForcesPub;
    ros::Publisher liftForcePub;
    ros::Publisher drugForcePub;
    ros::Publisher sideForcePub;

    ros::Publisher totalMomentPub;
    ros::Publisher aeroMomentPub;
    ros::Publisher controlSurfacesMomentPub;
    ros::Publisher aoaMomentPub;
    std::array<ros::Publisher, 5> motorsMomentsPub;
    ros::Publisher velocityPub;

    tf2_ros::TransformBroadcaster tfPub;
};

#endif  // SRC_RVIZ_VISUALIZATION_HPP_

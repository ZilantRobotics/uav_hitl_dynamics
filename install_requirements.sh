#!/bin/bash
sudo apt-get install -y ros-$ROS_DISTRO-mavros                                  \
                        ros-$ROS_DISTRO-mavlink                                 \
                        ros-$ROS_DISTRO-tf                                      \
                        ros-$ROS_DISTRO-tf2                                     \
                        ros-$ROS_DISTRO-tf2-ros                                 \
                        psmisc
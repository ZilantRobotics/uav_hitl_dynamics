ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO
LABEL description="Inno VTOL dynamics"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src


# 1. Install basic requirements
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update                          &&  \
    apt-get upgrade -y                      &&  \
    apt-get install -y  git python-catkin-tools python-pip python3-pip
RUN if [[ "$ROS_DISTRO" = "melodic" ]] ; then apt-get install -y python-pip python-catkin-tools ; fi

# 2. Install packages requirements
COPY install_requirements.sh install_requirements.sh
RUN ./install_requirements.sh

# 3. Install dependencies
RUN git clone https://github.com/InnopolisAero/uavcan_msgs.git /catkin_ws/src/uavcan_msgs
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws && catkin build

RUN git clone https://github.com/InnopolisAero/geographiclib_conversions.git /catkin_ws/src/geographiclib_conversions
RUN /catkin_ws/src/geographiclib_conversions/scripts/install.sh
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws && catkin build

# 4. Copy the source files
COPY cmake/             inno_vtol_dynamics/cmake/
COPY include/           inno_vtol_dynamics/include/
COPY libs/              inno_vtol_dynamics/libs/
COPY meshes/            inno_vtol_dynamics/meshes/
COPY msg/               inno_vtol_dynamics/msg/
COPY src/               inno_vtol_dynamics/src/
COPY urdf/              inno_vtol_dynamics/urdf/
COPY CMakeLists.txt     inno_vtol_dynamics/CMakeLists.txt
COPY package.xml        inno_vtol_dynamics/package.xml
COPY requirements.txt   inno_vtol_dynamics/requirements.txt

# 5. Build
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws && catkin build

CMD echo "main process has been started"                                        &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash                                      &&  \
    source /catkin_ws/devel/setup.bash                                          &&  \
    echo "container has been finished"
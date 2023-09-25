ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO
LABEL description="UAV VTOL dynamics"
SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/src


# 1. Install basic requirements
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y git python3-catkin-tools python3-pip

# 2. Install packages requirements
COPY install_requirements.sh install_requirements.sh
RUN ./install_requirements.sh

# 3. Install dependencies
RUN git clone https://github.com/RaccoonlabDev/geographiclib_conversions.git /catkin_ws/src/geographiclib_conversions
RUN /catkin_ws/src/geographiclib_conversions/scripts/install.sh
RUN source /opt/ros/$ROS_DISTRO/setup.bash && cd /catkin_ws && catkin build

# 4. Copy the source files
WORKDIR /catkin_ws/src/inno_vtol_dynamics
COPY libs/              libs/
COPY meshes/            meshes/
COPY src/               src/
COPY tests/             tests/
COPY urdf/              urdf/
COPY CMakeLists.txt     CMakeLists.txt
COPY package.xml        package.xml
COPY requirements.txt   requirements.txt
COPY config/ config/
COPY launch/ launch/
COPY catkin_test.sh catkin_test.sh

COPY config/ config/
COPY launch/ launch/
COPY catkin_test.sh catkin_test.sh
COPY sonar/ /sonar/
COPY sonar.sh sonar.sh

CMD echo "main process has been started"                                        &&  \
    source /opt/ros/$ROS_DISTRO/setup.bash                                      &&  \
    ./catkin_test.sh -d                                                         &&  \
    source /catkin_ws/devel/setup.bash                                          &&  \
    echo "container has been finished"

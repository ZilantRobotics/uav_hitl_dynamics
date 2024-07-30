cmake_minimum_required(VERSION 2.8.3)
project(innopolis_vtol_dynamics)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    mavros_msgs
    tf2
    tf2_ros
    message_generation
)

find_package(mavlink)

find_package(Eigen3 REQUIRED)

catkin_package(
    LIBRARIES innopolis_vtol_dynamics
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros roslib message_runtime
)


###########
## Build ##
###########
include_directories(
    src
    src/dynamics
    src/dynamics/vtol
    src/dynamics/multirotor
    src/dynamics/quadcopter
    src/dynamics/octocopter
    src/sensors
    ${catkin_INCLUDE_DIRS}
    ${mavlink_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    libs/UavDynamics/include
)

add_library(${PROJECT_NAME} src/dynamics/vtol/vtolDynamicsSim.cpp
                            src/dynamics/multirotor/multirotor.cpp
                            src/dynamics/quadcopter/quadcopter.cpp
                            src/dynamics/octocopter/octocopter.cpp
                            src/dynamics/uavDynamicsSimBase.cpp

                            libs/multicopterDynamicsSim/inertialMeasurementSim.cpp
                            libs/multicopterDynamicsSim/multicopterDynamicsSim.cpp
                            libs/UavDynamics/src/math/wmm.cpp
                            libs/UavDynamics/src/math/geodetic.cpp

                            src/actuators.cpp
                            src/common_math.cpp
                            src/cs_converter.cpp
                            src/logger.cpp
                            src/rviz_visualization.cpp
                            src/scenarios.cpp

                            src/sensors/attitude.cpp
                            src/sensors/barometer.cpp
                            src/sensors/battery.cpp
                            src/sensors/differential_pressure.cpp
                            src/sensors/esc_status.cpp
                            src/sensors/fuel_tank.cpp
                            src/sensors/gnss.cpp
                            src/sensors/ice.cpp
                            src/sensors/imu.cpp
                            src/sensors/velocity.cpp
                            src/sensors/mag.cpp
                            src/sensors/sensors.cpp
)

## 1. Declare a C++ uav_dynamics_node executable
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_include_directories(${PROJECT_NAME}_node
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

## 2. Declare a C++ mixer_node executable
include(src/mixers/CMakeLists.txt)

#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-inno-vtol-dynamics-test tests/test_vtol_dynamics.cpp)
if(TARGET ${PROJECT_NAME}-inno-vtol-dynamics-test)
  target_link_libraries(${PROJECT_NAME}-inno-vtol-dynamics-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(${PROJECT_NAME}-inno-vtol-dynamics-test
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
endif()

catkin_add_gtest(${PROJECT_NAME}-isa_model-test tests/test_isa_model.cpp)
if(TARGET ${PROJECT_NAME}-isa_model-test)
  target_link_libraries(${PROJECT_NAME}-isa_model-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(${PROJECT_NAME}-isa_model-test
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
endif()

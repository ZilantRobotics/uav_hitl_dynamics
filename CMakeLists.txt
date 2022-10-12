cmake_minimum_required(VERSION 2.8.3)
project(innopolis_vtol_dynamics)

# Default to building in release mode when no options are set
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    uavcan_msgs
    tf2
    tf2_ros
    geographiclib_conversions
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
    src/dynamics/quadcopter
    ${catkin_INCLUDE_DIRS}
    ${mavlink_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/dynamics/vtol/vtolDynamicsSim.cpp
                            src/dynamics/quadcopter/flightgogglesDynamicsSim.cpp
                            src/dynamics/uavDynamicsSimBase.cpp
                            libs/multicopterDynamicsSim/inertialMeasurementSim.cpp
                            libs/multicopterDynamicsSim/multicopterDynamicsSim.cpp
                            src/sensors.cpp
                            src/common_math.cpp
                            src/cs_converter.cpp
)

## 1. Declare a C++ uav_dynamics_node executable
add_executable(${PROJECT_NAME}_node src/uavDynamicsNode.cpp)
target_include_directories(${PROJECT_NAME}_node
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

## 2. Declare a C++ inno_vtol_reverse_mixer_node executable
add_executable(${PROJECT_NAME}_inno_vtol_reverse_mixer_node src/inno_vtol_reverse_mixer_node.cpp)
target_include_directories(${PROJECT_NAME}_inno_vtol_reverse_mixer_node
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
set_target_properties(${PROJECT_NAME}_inno_vtol_reverse_mixer_node PROPERTIES OUTPUT_NAME inno_vtol_reverse_mixer_node PREFIX "")
add_dependencies(${PROJECT_NAME}_inno_vtol_reverse_mixer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_inno_vtol_reverse_mixer_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-inno-vtol-dynamics-test src/tests/test_vtol_dynamics.cpp)
if(TARGET ${PROJECT_NAME}-inno-vtol-dynamics-test)
  target_link_libraries(${PROJECT_NAME}-inno-vtol-dynamics-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(${PROJECT_NAME}-inno-vtol-dynamics-test
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
endif()

catkin_add_gtest(${PROJECT_NAME}-isa_model-test src/tests/test_isa_model.cpp)
if(TARGET ${PROJECT_NAME}-isa_model-test)
  target_link_libraries(${PROJECT_NAME}-isa_model-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(${PROJECT_NAME}-isa_model-test
                BEFORE
                PUBLIC ${MAVLINK_INCLUDE_DIRS})
endif()
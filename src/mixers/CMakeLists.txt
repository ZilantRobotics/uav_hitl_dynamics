cmake_minimum_required(VERSION 2.8.3)

set(EXECUTABLE ${PROJECT_NAME}_mixer_node)

add_executable(${EXECUTABLE}
    src/mixers/main.cpp
    src/mixers/base_mixer.cpp
    src/mixers/mixer_babyshark.cpp
    src/mixers/px4_v1.12.1_13070.cpp
    src/mixers/px4_v1.14.0_13000_vtol_4_motors.cpp
    src/mixers/px4_v1.14.0_13000_vtol_8_motors.cpp
    src/mixers/mixer_direct.cpp
)

target_include_directories(${EXECUTABLE} BEFORE PUBLIC ${MAVLINK_INCLUDE_DIRS})
set_target_properties(${EXECUTABLE} PROPERTIES OUTPUT_NAME mixer_node PREFIX "")
add_dependencies(${EXECUTABLE} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${EXECUTABLE}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

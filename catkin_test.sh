#!/bin/bash

set -e
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

stop_ros() {
    killall -9 roscore
    killall -9 rosmaster
}

# Setup ROS environment variables if inside a container
key="$1"
case $key in
    -d|--docker)
        source /opt/ros/$ROS_DISTRO/setup.bash
        source /catkin_ws/devel/setup.bash
        ;;
*)
    ;;
esac

# Run tests
roscore &
roslaunch innopolis_vtol_dynamics load_parameters.launch --wait
catkin run_tests --no-deps innopolis_vtol_dynamics

# catkin_tools issue #245. catkin run_tests may return 0 when fail.
# https://github.com/catkin/catkin_tools/issues/245
# Check report directory to verify that tests are ok.
roscd && cd ../build/
CATKIN_TESTS_FAILURE_RESULTS_PATH=innopolis_vtol_dynamics/test_results/innopolis_vtol_dynamics/MISSING-*
if [ ! -z "$(ls -A $CATKIN_TESTS_FAILURE_RESULTS_PATH)" ]; then
    echo "Failure results have been found. Exit with error."
    stop_ros
    exit -1
fi

stop_ros

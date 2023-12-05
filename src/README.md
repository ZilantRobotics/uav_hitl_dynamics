# Inno VTOL Dynamics

This directory contains source code for a simulator of various UAV Dynamics, including quadcopters, octocopters, and VTOL (Vertical Take-Off and Landing) plane models. The simulator includes several sensor simulations and a set of dynamics models. The simulator is designed to interoperate with the ROS (Robot Operating System) ecosystem.

## Folder Structure and Contents

1. **[main](./main.hpp)**: Handles initiation and execution of the UAV dynamics simulator. Sets up ROS node, initializes various components (sensors, actuators, dynamics simulators, RViz visualization, logger), manages the simulation clock, starts threads for processing dynamics, publishing to ROS, and logging. The dynamics simulator used and other parameters are fetched from ROS parameters. It processes the UAV dynamics based on actuator inputs and the current scenario. Calibration mode can be triggered via a ROS topic.

2. **[dynamics](./dynamics/README.md)**: This folder contains different UAV dynamics models. The details for each type of UAV dynamics model can be found in their respective README files:
   - [multirotor](./dynamics/multirotor/README.md)
   - [octocopter](./dynamics/octocopter/README.md)
   - [quadcopter](./dynamics/quadcopter/README.md)
   - [vtol](./dynamics/vtol/README.md)

3. **[sensors](./sensors/README.md)**: Contains various sensor simulations. More details about the sensors can be found in the [sensors README](./sensors/README.md).

4. **[actuators](./actuators.hpp)**: Contains implementation of UAV actuators simulation. This module handles actuator input and the armed state of the UAV. It updates actuator states upon receiving new `sensor_msgs::Joy` messages on the `/uav/actuators` topic, records timestamps, computes message delays, and adjusts throttle according to `_scenarioType`. It also handles arming/disarming by listening to `std_msgs::Bool` messages on the `/uav/arm` topic and logs any state changes.

5. **[mixers/main.cpp](mixers/main.cpp)**: This node handles the reverse mixing operation for VTOL UAVs. It allows the conversion from raw actuator commands to a specific UAV's actuator command mapping. This mapping differs based on the type of VTOL (e.g., BabyShark, Inno VTOL, etc.). The reverse mixer node utilizes classes defined for each VTOL type, i.e., `BabysharkReverseMixer`, `InnoVtolReverseMixer`, and `DirectMixer`, each inheriting from the base class `BaseReverseMixer`. The node operates on the ROS (Robot Operating System) framework, and uses sensor_msgs/Joy message type to handle the actuator command data. It subscribes to raw actuator commands and publishes the mapped actuator commands based on the chosen mixer model. The chosen mixer model is defined by the "mixer" parameter in the ROS parameter server. If the mixer is not defined or invalid, the node will terminate with an error message. [**mixers.md**](./mixers.md) contains description of the mixer implementation.

6. **[scenarios](./scenarios.hpp)**: Includes different simulation scenarios (sensors fails).

7. **[common_math](./common_math.hpp)**: Includes mathematical operations and utilities used in the project.

8. **[cs_converter](./cs_converter.hpp)**: Includes the implementation of coordinate system conversion utilities.

9. **[logger](./logger.hpp)**: Contains the logging functionalities.

10. **[rviz_visualization](./rviz_visualization.hpp)**: Contains utilities for visualizing simulation results in RViz.

11.  **tests**: Contains tests (for the ISA model and VTOL dynamics currently).

Please refer to the README files in each subdirectory for a detailed explanation of each component.
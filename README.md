# inno_vtol_dynamics [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=PonomarevDA_inno_vtol_dynamics&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=PonomarevDA_inno_vtol_dynamics) [![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=PonomarevDA_inno_vtol_dynamics&metric=code_smells)](https://sonarcloud.io/summary/new_code?id=PonomarevDA_inno_vtol_dynamics) [![Lines of Code](https://sonarcloud.io/api/project_badges/measure?project=PonomarevDA_inno_vtol_dynamics&metric=ncloc)](https://sonarcloud.io/summary/new_code?id=PonomarevDA_inno_vtol_dynamics) [![Coverage](https://sonarcloud.io/api/project_badges/measure?project=PonomarevDA_inno_vtol_dynamics&metric=coverage)](https://sonarcloud.io/summary/new_code?id=PonomarevDA_inno_vtol_dynamics)

This package is the core of the [UAV HITL dynamics simulator](https://github.com/InnopolisAero/innopolis_vtol_dynamics). It has an implementation of custom quadcopter vertical takeoff and landing aircraft dynamics.

![VTOL plane](img/inno_vtol.png?raw=true "VTOL plane")

Innopolis VTOL has UAVCAN onboard electronics based on [RaccoonLab Cyphal/DroneCAN sensors and actuators](https://raccoonlab.co/store) (so it's a good example of full UAVCAN-based onboard control and object for new UAVCAN-HITL simulation approach).

This package contains Innopolis VTOL simulation based on rigid body kinematics and dynamics, CFD analysis and actuators simulation.

![Innopolis VTOL plane dynamics structure](img/structure.jpeg?raw=true "Innopolis VTOL plane dynamics structure")

The package is used inside a new Cyphal/DroneCAN-HITL . 

The node from this package communicates with the flight stack via communicator `UAV dynamics` by subscribing and publishing to the following topics:


```mermaid
flowchart LR

actuators[ /uav/actuators, sensor_msgs/Joy] --> F(uav_hitl_node)
arm[ /uav/arm, std_msgs/Bool] --> F(uav_hitl_node)
calibration[ /uav/calibration, std_msgs/UInt8] --> F(uav_hitl_node)
scenario[ /uav/scenario, std_msgs/UInt8] --> F(uav_hitl_node)
F --> temperature[ /uav/static_temperature, std_msgs/Float32]
F --> static_pressure[ /uav/static_pressure, std_msgs/Float32]
F --> raw_air_data[ /uav/raw_air_data, std_msgs/Float32]
F --> gps_point[ /uav/gps_point, sensor_msgs/NavSatFix]
F --> velocity[ /uav/velocity, geometry_msgs/Twist]
F --> imu[ /uav/imu, sensor_msgs/Imu]
F --> mag[ /uav/mag, sensor_msgs/MagneticField]
F --> esc_status[ /uav/esc_status, mavros_msgs/ESCTelemetryItem]
F --> ice_rpm[ /uav/esc_status, mavros_msgs/ESCStatusItem]
F --> ice_status[ /uav/ice_status, std_msgs/UInt8]
F --> fuel_tank_status[ /uav/ice_status, std_msgs/UInt8]
F --> battery_status[ /uav/battery_status, sensor_msgs/BatteryState]
```

Auxilliary topics might be enabled/disabled in the [sim_params.yaml](uav_dynamics/inno_vtol_dynamics/config/sim_params.yaml) config file. You may implement your own sensors in the [sensors.cpp](uav_dynamics/inno_vtol_dynamics/src/sensors/sensors.cpp) file.

To work in pair with [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) as physics engine via [inno_sim_interface](https://github.com/RaccoonlabDev/inno_sim_interface) it publishes and subscribes on following topics.

```mermaid
flowchart LR

F(uav_hitl_node) --> actuators[ /uav/actuators, sensor_msgs/Joy]
F --> gps_point[ /uav/gps_point, sensor_msgs/NavSatFix]
F --> velocity[ /uav/velocity, geometry_msgs/Twist]
F --> attitude[ /uav/attitude, geometry_msgs/QuaternionStamped]
actuators --> G(inno_sim_interface)
gps_point --> G
velocity --> G
attitude --> G
```

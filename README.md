# inno_vtol_dynamics

This package is the core of the [UAV HITL dynamics simulator](https://github.com/InnopolisAero/innopolis_vtol_dynamics). It has an implementation of custom quadcopter vertical takeoff and landing aircraft dynamics.

![VTOL plane](img/inno_vtol.png?raw=true "VTOL plane")

Innopolis VTOL has UAVCAN onboard electronics based on [RaccoonLab UAVCAN sensors and actuators](http://raccoonlab.org/uavcan) (so it's a good example of full UAVCAN-based onboard control and object for new UAVCAN-HITL simulation approach).

This package contains Innopolis VTOL simulation based on rigid body kinematics and dynamics, CFD analysis and actuators simulation.

![Innopolis VTOL plane dynamics structure](img/structure.jpeg?raw=true "Innopolis VTOL plane dynamics structure")

The package is used inside a new UAVCAN-HITL . 

The node from this package communicates with the flight stack via communicator `UAV dynamics` by subscribing and publishing to the following topics:

| № | Subscribed topics | msg                                   |
| - | ----------------- | ------------------------------------- |
| 1 | /uav/actuators    | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) |
| 2 | /uav/arm          | [std_msgs::Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html) |

Minimal sensors publishers list of topics:

| № | Advertised topics         | msg                                   |
| - | -------------------------- | ------------------------------------- |
| 1 | /uav/static_temperature    | uavcan_msgs/StaticTemperature         |
| 2 | /uav/static_pressure       | uavcan_msgs/StaticPressure            |
| 3 | /uav/raw_air_data          | uavcan_msgs/RawAirData                |
| 4 | /uav/gps_position          | uavcan_msgs/Fix                       |
| 5 | /uav/imu                   | [sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)                      |
| 6 | /uav/mag                   | [sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html)  |

Extended sensors publishers list of topics:

| № | Advertised topics         | msg                                   |
| - | -------------------------- | ------------------------------------- |
| 7 | /uav/esc_status            | uavcan_msgs/EscStatus                 |
| 8 | /uav/ice_status            | uavcan_msgs/IceReciprocatingStatus                       |
| 9 | /uav/fuel_tank_status      | uavcan_msgs/IceFuelTankStatus             |
| 10| /uav/battery_status        | [sensor_msgs/BatteryState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/BatteryState.html)    |

Here topics 1-6 are necessary for any simulation. The last 4 topics are auxiliary and you may enable/disable them in the [sim_params.yaml](uav_dynamics/inno_vtol_dynamics/config/sim_params.yaml) config file. You may implement your own sensors in the [sensors.cpp](uav_dynamics/inno_vtol_dynamics/src/sensors.cpp) file.

To work in pair with [InnoSimulator](https://github.com/inno-robolab/InnoSimulator) as physics engine via [inno_sim_interface](https://github.com/InnopolisAero/inno_sim_interface) it publishes and subscribes on following topics.

| № | Advertised topics | msg                             |
| - | ----------------- | ------------------------------- |
| 1 | /uav/actuators    | [sensor_msgs/Joy](https://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html)                 |
| 2 | /uav/gps_position | uavcan_msgs/Fix                 |
| 3 | /uav/attitude     | geometry_msgs/QuaternionStamped |
# Mixer discription

Cyphal/DroneCAN/Mavlink communicators just directly publish what they reveice from their interface to ROS. They doesn't have any prior information about the airframe.

Intefaces can have different output. It is especially true for MAVLink interface because it is based on specific [HIL_ACTUATOR_CONTROLS](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS) message.

At the same time dynamics expect the setpoint input in their own notation and with their own rules.

So, we need an intermediate layer to bring everything together. Here it is called Mixer.

The typical workflow of setpint is the following:

1. Cyphal/DroneCAN/MAVLink communicator gets setpoint and publishes it to `/uav/actuators_raw` as `sensor_msgs/Joy`
2. Reverse Mixer receives the command, fix it and publishes it to `/uav/actuators` as `sensor_msgs::Joy`
3. The dynamics receives the command and may convert to his own internal representation if it is necessary.

> What about arming state?

There are 3 types of mixers:
1. Direct mixer is the most simplest. It just copy data raw date to the actual date
2. babyshark_standard_vtol_mixer fix input with respect to the Babyshark airframe
3. inno_vtol_mixer is similar to the previous one

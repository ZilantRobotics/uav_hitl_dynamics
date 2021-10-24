# inno_vtol_dynamics

The core of Innopolis VTOL dynamics simulation for innopolis_vtol_dynamics simulator

Innopolis VTOL is a custom quadcopter vertical takeoff and landing aircraft.
![Innopolis VTOL plane](img/inno_vtol.png?raw=true "Innopolis VTOL plane")

Innopolis VTOL has UAVCAN onboard electronics based on [RaccoonLab UAVCAN sensors and actuators](http://raccoonlab.org/uavcan) (so it's a good example of full UAVCAN-based onboard control and object for new UAVCAN-HITL simulation approach).

This package contains Innopolis VTOL simulation based on rigid body kinematics and dynamics, CFD analysis and actuators simulation.

![Innopolis VTOL plane dynamics structure](img/structure.jpeg?raw=true "Innopolis VTOL plane dynamics structure")

The package is used inside a new UAVCAN-HITL [Innopolis VTOL dynamics simulator](https://github.com/InnopolisAero/innopolis_vtol_dynamics). 

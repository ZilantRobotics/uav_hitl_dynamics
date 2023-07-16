# Quadcopter Dynamics

This code defines a QuadcopterDynamics class which is inherited from the MultirotorDynamics base class. Here are the specific aspects:

1. `initStaticMotorTransform` function: This function accepts a moment arm and sets up the locations and orientations of the four motors (quadrotor configuration) in the body frame of the quadcopter.

2. `mapCmdActuator` function: It converts the motor commands from the PX4 convention (used in the ROS ecosystem) to the internal representation used by the `MulticopterDynamicsSim` simulator. 

PX4 is based on FRD notation and has the following motors enumeration:

<img src="https://dev.px4.io/master/assets/airframes/types/QuadRotorX.svg" alt="drawing" width="150">

The dynamics library is based on FLU notation and has the following motors enumeration:

|||||
|-|-|-|-|
| 0 | front left  | positive moment around | +0.08, +0.08
| 1 | tail left   | negative moment around | -0.08, +0.08
| 2 | tail right  | positive moment around | -0.08, -0.08
| 3 | front right | negative moment around | +0.08, -0.08


This specific ordering suits the requirements of the `MulticopterDynamicsSim` simulation environment requirements. Also, this code assumes that the incoming command vector has the correct number of elements and does not perform any bounds checking.


# Mathematical Model

`QuadcopterDynamics` is a simulator for quadcopters based on `MulticopterDynamicsSim`. `MulticopterDynamicsSim` provides a general mathematical model applicable to multicopters, i.e., drones with an arbitrary number of rotors. For the specific case of a quadcopter, `QuadcopterDynamics` utilizes the capabilities provided by `MulticopterDynamicsSim`.

The simulation enables comprehensive investigation of quadcopter behavior under various conditions and control strategies. It captures the physics of quadcopter motion in a 3D environment, taking into account factors like gravitational force, aerodynamic forces, motor dynamics, and external disturbances.

For the detailed description of basic mathematical model used, please, check [multirotor/README.md, Mathematical Model Section](../multirotor/README.md)

# Parameters and Configurating

Please refer to [`MultirotorDynamics` README.md](../multirotor/README.md) `Drone Simulation Configuration` Section.
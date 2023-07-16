# MultirotorDynamics

`MultirotorDynamics` is an intermediate base class designed to provide a unified interface for different types of multirotor drones such as quadcopters and octocopters. It is an abstract class inheriting from the `UavDynamicsSimBase` class, and it's designed to be extended by classes that implement the specific dynamics of different types of drones. 

The class utilizes the `MulticopterDynamicsSim` library to simulate the core dynamics of a multirotor aircraft, which includes kinematics, dynamics, aerodynamics, environmental factors, and motor models.

Key components of the `MultirotorDynamics` class include:

- Initialization (`init()`): The `init()` method retrieves the aerodynamic coefficients and other parameters for the multirotor from ROS parameters. These parameters are then used to instantiate the `MulticopterDynamicsSim` object.

- State update (`process()`): The `process()` method advances the simulation of the multirotor's state. It takes as input the commanded motor speeds and a time step, and updates the internal state of the `MulticopterDynamicsSim` object accordingly.

- State retrieval: There are several methods to retrieve the state of the multirotor, including its position (`getVehiclePosition()`), attitude (`getVehicleAttitude()`), linear velocity (`getVehicleVelocity()`), and angular velocity (`getVehicleAngularVelocity()`).

- IMU measurements (`getIMUMeasurement()`): This method retrieves simulated measurements from the multirotor's IMU.

The class also includes two abstract methods (`initStaticMotorTransform()` and `mapCmdActuator()`) that must be implemented by any derived classes. These methods are used for setting up the physical layout of the motors and mapping actuator commands to specific motors.

Overall, `MultirotorDynamics` provides a versatile base for creating simulations of different types of multirotor aircraft. By separating out the specifics of motor arrangement and command mapping, it allows for code reuse and a clean, maintainable design.


For the detailed description of basic mathematical model used, please, check [quadcopter/README.md, Mathematical Model Section](../quadcopter/README.md)
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


# Drone Simulation Configuration 

## Overview

The `MultirotorDynamics` class is central to the configuration of the drone simulation. This class initiates various parameters related to the physical properties and dynamics of the drone, based on data fetched from the ROS parameters. 

## Simulation Parameters

Parameters are fetched using the `getParameter` function, which checks for the existence of a specific parameter in the ROS parameter server, and if not found, defaults to a given value. The parameters include:

1. **Vehicle parameters**: These define the physical properties of the drone.
   - `vehicle_mass`: The mass of the drone in kg.
   - `motor_time_constant`: The time constant of the motor in sec.
   - `motor_rotational_inertia`: The rotational inertia of the motor in kg m^2.
   - `thrust_coefficient`: The thrust coefficient in N/(rad/s)^2.
   - `torque_coefficient`: The torque coefficient in Nm/(rad/s)^2.
   - `drag_coefficient`: The drag coefficient in N/(m/s).

2. **Aerodynamic parameters**: These define the behavior of the drone in flight.
   - `aeromoment_coefficient_xx`, `aeromoment_coefficient_yy`, `aeromoment_coefficient_zz`: The aerodynamic moment coefficients.
   - `vehicle_inertia_xx`, `vehicle_inertia_yy`, `vehicle_inertia_zz`: The inertia of the drone.

3. **Propeller parameters**: These define the properties and limitations of the propellers.
   - `max_prop_speed`: The maximum propeller speed in rad/s.
   - `moment_process_noise`: The process noise of the moment in (Nm)^2 s.
   - `force_process_noise`: The process noise of the force in N^2 s.

4. **Gravity and moment parameters**: These define environmental and structural properties of the drone.
   - `gravity`: The force of gravity, set to Earth's gravity (0., 0., -9.81).
   - `moment_arm`: The distance between the center of mass and the center of each motor, or the length of the arm that holds the propeller.

5. **IMU parameters**: These define the properties and noise levels of the drone's Inertial Measurement Unit (IMU).
   - `accelerometer_biasprocess`, `gyroscope_biasprocess`: The process noise auto-correlation for the accelerometer and gyroscope.
   - `accelerometer_biasinitvar`, `gyroscope_biasinitvar`: The initial variance of the accelerometer and gyroscope bias.
   - `accelerometer_variance`, `gyroscope_variance`: The measurement noise variance for the accelerometer and gyroscope.

## Simulation Initialization

During initialization, the `MultirotorDynamics::init()` function is called, which initializes the multicopter simulator using the fetched parameters. It also sets the motor speed and initializes the IMU parameters.

## Running the Simulation

The `MultirotorDynamics::process()` function is used to update the drone's state based on the input motor speed commands. The drone's position, attitude, velocity, angular velocity, and IMU measurements can be fetched using their respective getter functions.

## Adjusting Simulation Parameters

To adjust the drone simulation parameters, edit the corresponding entries in the ROS parameter server. Please note that changing these parameters will significantly affect the drone's behavior in the simulation. As such, it's recommended to understand the impact of each parameter before making any adjustments.
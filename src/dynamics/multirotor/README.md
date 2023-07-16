# 1 MultirotorDynamics

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

# 2 Mathematical Model

`MultirotorDynamics` is a high-fidelity simulator for copters based on `MulticopterDynamicsSim`. `MulticopterDynamicsSim` provides a general mathematical model applicable to multicopters, i.e., drones with an arbitrary number of rotors.

The simulation enables comprehensive investigation of copter behavior under various conditions and control strategies. It captures the physics of copter motion in a 3D environment, taking into account factors like gravitational force, aerodynamic forces, motor dynamics, and external disturbances.

The `MulticopterDynamicsSim` class comprises a comprehensive mathematical model that governs the behavior of the multicopter system. It includes the kinematic and dynamic equations of motion, aerodynamics, environment model, and a motors model.

## 2.1 Acknowledgments & Citations

This project uses the `MulticopterDynamicsSim` code developed and maintained as part of the FlightGoggles project by the team at the Massachusetts Institute of Technology (MIT). We are deeply thankful for their contributions to the open-source community.

We encourage anyone who benefits from our work, which utilizes `MulticopterDynamicsSim`, to also cite the original authors:

```bibtex
@misc{1905.11377,
  Title = {FlightGoggles: Photorealistic Sensor Simulation for Perception-driven Robotics using Photogrammetry and Virtual Reality},
  Author = {Winter Guerra and Ezra Tal and Varun Murali and Gilhyun Ryou and Sertac Karaman},
  Year = {2019},
  Eprint = {arXiv:1905.11377},
}
```

For more information about FlightGoggles and the team's work, please follow these links: [Paper](https://arxiv.org/abs/1905.11377), [Website](http://flightgoggles.mit.edu), [GitHub](https://github.com/mit-fast/FlightGoggles).


## 2.2 Kinematics

The kinematic model describes the position and orientation of the drone. Position in the world-fixed reference frame is described by a 3D vector $\vec{p} = [x, y, z]^\top$. The orientation is given by a unit quaternion $\mathbf{q} = [q_w, q_x, q_y, q_z]^\top$, where $q_w$ is the scalar part and $[q_x, q_y, q_z]^\top$ is the vector part.

The rate of change of position is the velocity $\vec{v} = [v_x, v_y, v_z]^\top$ in the world-fixed frame.

The rate of change of the quaternion, which describes the drone's rotational motion, is given by the formula

```math
\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\omega}_q
```

where ${\boldsymbol{\omega}}_q = [0, \omega_x, \omega_y, \omega_z]^\top$ is a quaternion representation of the angular velocity vector $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^\top$ in the drone-fixed frame, and $\otimes$ denotes the quaternion multiplication operation.

## 2.3 Dynamics

The dynamic model describes the motion of the drone due to forces and torques. According to Newton's second law, the force on an object is equal to its mass times its acceleration. In the world-fixed frame, this gives:

```math
m \vec{a} = \vec{F}_{\text{gravity}} + \vec{F}_{\text{thrust}} + \vec{F}_{\text{drag}} + \vec{F}_{\text{stoch}}
```

where $m$ is the mass of the drone, $\vec{a}$ is the acceleration, and $F_{\text{gravity}}$, $F_{\text{thrust}}$, $F_{\text{drag}}$, and $F_{\text{stoch}}$ are the gravitational, thrust, aerodynamic drag, and stochastic forces, respectively.

The rotational dynamics are governed by the Euler's rotation equations:

$$
\mathbf{J} \dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega} \times (\mathbf{J} \boldsymbol{\omega} + \vec{h}_{\text{motor}})
$$

where $\mathbf{J}$ is the moment of inertia matrix, $\dot{\boldsymbol{\omega}}$ is the angular acceleration, $\boldsymbol{\tau}$ is the total control moment, and $\vec{h}_{\text{motor}}$ is the total angular momentum of the motors.

## 2.4 Aerodynamics

The aerodynamic model calculates the drag forces and moments acting on the drone. The aerodynamic force is generally modeled as being proportional to the square of the velocity, and the aerodynamic moment is proportional to the square of the angular velocity.

### 2.4.1 Aerodynamic Force

The aerodynamic drag force is calculated by the function `getDragForce` as:

$$
\vec{F}_{\text{drag}} = -C_d * \vec{v}
$$

where $\vec{v}$ is the velocity vector of the drone in the world-fixed frame, and $C_d$ is the drag coefficient vector. The drag force is assumed to be directly proportional to the velocity, opposing the direction of motion. The coefficient $C_d$ is a 3x3 diagonal matrix where the diagonal elements are the coefficients of drag along the x, y, and z axes, respectively. 

### 2.4.2 Aerodynamic Moment

The aerodynamic moment or torque, is calculated by the function `getAeroMoment` as:

$$
\boldsymbol{\tau}_{\text{aero}} = -C_q * \boldsymbol{\omega}
$$

where $\boldsymbol{\omega}$ is the angular velocity vector of the drone in the drone-fixed frame, and $C_q$ is the angular drag coefficient vector. Like the force, the moment is assumed to be directly proportional to the angular velocity, opposing the rotation. The coefficient $C_q$ is a 3x3 diagonal matrix where the diagonal elements are the coefficients of angular drag around the x, y, and z axes, respectively. 

These are simplified models for the aerodynamic forces and moments, which are generally a function of the square of velocity and angular velocity, respectively. However, for small velocities, the linear approximation is often used for simplicity.

## 2.5 Environment Model

The environment model accounts for the effects of gravity and any additional stochastic forces and moments due to wind or other unmodeled disturbances.

The gravitational force is given by $\vec{F}_{\text{gravity}} = m \vec{g}$, where $\vec{g} = [0, 0, -g]^\top$ is the acceleration due to gravity.

The stochastic forces and moments, $F_{\text{stoch}}$ and ${\tau}_{\text{stoch}}$, are random variables that represent wind and other unmodeled effects.

## 2.6 Motors Model

Each motor generates a thrust force proportional to the square of its speed, and an opposite reaction moment proportional to the thrust and the motor's direction. The dynamics of each motor are modeled as a first-order system with a time constant.

In addition, each motor contributes to the total angular momentum of the drone. The angular momentum of a motor is proportional to its speed and its rotational inertia, and it is directed opposite to the motor's direction.

The motor forces and moments are transformed from the motor frames to the drone body frame using transformation matrices.

### 2.6.1 Motor Speed Dynamics

The derivative of motor speed is calculated by the function `getMotorSpeedDerivative` as:

```math
\dot{\omega}_{\text{motor}_i} = \frac{\omega_{\text{cmd}_i} - \omega_{\text{motor}_i}}{T_{\text{motor}_i}}
```

where $\omega_{cmd_i}$ is the commanded speed of motor i, $\omega_{motor_i}$ is the actual speed of motor i, and $T_{motor_i}$ is the time constant for motor i. This equation models the motor speed as a first order system response to a step command, with a time constant defined for each motor. 

### 2.6.2 Motor Thrust

The total thrust provided by the motors is calculated by the function `getThrust` as:

```math
\vec{T} = \sum_{i=1}^{\text{numCopter}} k_{\text{thrust}_i} \cdot \omega_{\text{motor}_i}^2 \cdot \hat{e}_z
```

where $k_{thrust_i}$ is the thrust coefficient for motor i, $\omega_{motor_i}$ is the speed of motor i, and $\hat{e}_z$ is the unit vector along the z-axis (downwards). This equation sums up the thrust provided by each motor (assumed to be proportional to the square of the motor speed), all in the z direction (downwards).

### 2.6.3 Control Moment

The control moment provided by the motors is calculated by the function `getControlMoment` as:

```math
\boldsymbol{\tau}_{\text{control}} = \sum_{i=1}^{\text{numCopter}} (k_{\text{thrust}_i} \cdot \omega_{\text{motor}_i}^2 \cdot r_i \times \hat{e}_z - k_{\text{moment}_i} \cdot \dot{\omega}_{\text{motor}_i} \cdot \hat{e}_z)
```

where $k_{thrust_i}$ is the thrust coefficient for motor i, 
$\omega_{motor_i}$ is the speed of motor i, 
$\dot{\omega_{motor_i}}$ is the acceleration of motor i, 
$k_{moment_i}$ is the moment coefficient for motor i, 
$r_i$ is the position of motor i with respect to the center of mass of the drone, 
and $\hat{e}_z$ is the unit vector along the z-axis (downwards). 
This equation computes the net moment created by the thrusts and accelerations of the motors. The first term in the sum represents the moment due to the thrust force (force times distance), while the second term represents the moment due to the change in motor speed (moment of inertia times angular acceleration). 

Please note that these are highly simplified models of the motor dynamics and in reality, these can be much more complex including effects of current, voltage, temperature, magnetic fields and more. The simplicity of this model makes it computationally efficient and good enough for our applications.

# 3 Drone Simulation Configuration 

## 3.1 Overview

The `MultirotorDynamics` class is central to the configuration of the drone simulation. This class initiates various parameters related to the physical properties and dynamics of the drone, based on data fetched from the ROS parameters. 

## 3.2 Simulation Parameters

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

## 3.3 Simulation Initialization

During initialization, the `MultirotorDynamics::init()` function is called, which initializes the multicopter simulator using the fetched parameters. It also sets the motor speed and initializes the IMU parameters.

## 3.4 Running the Simulation

The `MultirotorDynamics::process()` function is used to update the drone's state based on the input motor speed commands. The drone's position, attitude, velocity, angular velocity, and IMU measurements can be fetched using their respective getter functions.

## 3.5 Adjusting Simulation Parameters

To adjust the drone simulation parameters, edit the corresponding entries in the ROS parameter server. Please note that changing these parameters will significantly affect the drone's behavior in the simulation. As such, it's recommended to understand the impact of each parameter before making any adjustments.
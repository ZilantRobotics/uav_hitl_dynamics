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


# 2 Mathematical Model

`QuadcopterDynamics` is a high-fidelity simulator for quadcopters based on `MulticopterDynamicsSim`. `MulticopterDynamicsSim` provides a general mathematical model applicable to multicopters, i.e., drones with an arbitrary number of rotors. For the specific case of a quadcopter, `QuadcopterDynamics` utilizes the capabilities provided by `MulticopterDynamicsSim`.

The simulation enables comprehensive investigation of quadcopter behavior under various conditions and control strategies. It captures the physics of quadcopter motion in a 3D environment, taking into account factors like gravitational force, aerodynamic forces, motor dynamics, and external disturbances.

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


# Parameters and Configurating

Please refer to [`MultirotorDynamics` README.md](../multirotor/README.md) `Drone Simulation Configuration` Section.
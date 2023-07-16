# Octorotor Coaxial dynamics

PX4 is based on FRD notation and has the following motors enumeration:

<img src="https://dev.px4.io/master/assets/airframes/types/OctoRotorXCoaxial.svg" alt="drawing" width="150">

The dynamics library is based on FLU notation and has the following motors enumeration:

|||||
|-|-|-|-|
| 0 | front left  | positive moment around | +0.08, +0.08
| 1 | tail left   | negative moment around | -0.08, +0.08
| 2 | tail right  | positive moment around | -0.08, -0.08
| 3 | front right | negative moment around | +0.08, -0.08
| 4 | front left  | positive moment around | +0.08, +0.08
| 5 | tail left   | negative moment around | -0.08, +0.08
| 6 | tail right  | positive moment around | -0.08, -0.08
| 7 | front right | negative moment around | +0.08, -0.08

# Mathematical Model

For the detailed description of basic mathematical model used, please, check [quadcopter/README.md, Mathematical Model Section](../quadcopter/README.md)
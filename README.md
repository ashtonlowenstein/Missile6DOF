# 6-DOF Flight Dynamics Simulator with GNC

## Overview
A modular 6-DOF rigid-body flight simulator written in C++, featuring:
- quaternion-based attitude dynamics
- thrust vector control (pitch & yaw)
- actuator dynamics with rate limits
- variable mass and inertia
- gravity modeling
- closed-loop attitude control (GNC stack)

## Features
- 6-DOF rigid body simulation
- modular force/moment models (gravity, aero, propulsion)
- Runge-Kutta 4 integrator
- thrust vector control with gimbal actuators
- simple GNC stack:
    - truth navigation
    - attitude hold guidance
    - quaternion PD controller

## Project Structure
- `sim_core`: simulation and GNC library
- `examples/`: runnable scenarios
- `tests/`: unit tests
- `docs/`: design and validation notes

## Example Scenarios
- `propulsion_only`: thrust validation
- `tvc_pitch_yaw`: off-axis thrust tests
- `attitude_hold`: closed-loop attitude stabilization

## Future Work
- aerodynamic model integration
- sensor modeling and EKF
- trajectory guidance
- improved control allocation

**Author**:
Ashton Lowenstein
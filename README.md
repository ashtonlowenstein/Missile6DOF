# 6-DOF Flight Dynamics Simulator with GNC

## Overview
A modular 6-DOF rigid-body flight simulator written in C++, featuring:
- quaternion-based attitude dynamics
- thrust vector control (pitch & yaw)
- actuator dynamics with rate limits
- variable mass and inertia
- gravity modeling
- aerodynamics modeling
- sensor modeling and state estimation
- closed-loop attitude control (GNC stack)

## Features
- 6-DOF rigid body simulation
- modular force/moment models (gravity, aero, propulsion)
- Runge-Kutta 4 integrator
- thrust vector control with gimbal actuators
- GNC stack:
    - EKF & dead reckoning navigation
    - attitude hold guidance
    - quaternion PD controller
- accelerometer and gyroscope model

## Project Structure
- `sim_core`: simulation and GNC library
- `examples/`: runnable scenarios
- `tests/`: unit tests
- `docs/`: design and validation notes

## Example Scenarios
- `IMU_simple_thrust`: dead reckoning & thrust validation
- `EKF_inertial_coords`: EKF validation
- `attitude_hold`: closed-loop attitude stabilization

## Future Work
- trajectory guidance
- improved control allocation

**Author**:
Ashton Lowenstein
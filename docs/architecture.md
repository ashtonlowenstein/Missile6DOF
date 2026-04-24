## System Architecture

At each timestep:

1. Navigation estimates the current state
2. Guidance computes the desired attitude and throttle
3. Control computes actuator commands
4. Dynamics computes forces and moments
5. Integrator advances the state

## Core Data Structures
**`State`**  
- `pos_inertial`: Inertial position
- `vec_inertial`: Inertial velocity
- `q_BI`: Quaternion attitude (body -> inertial rotation)
- `omega_body`: Body angular momentum
- `mass`: Total mass

**`Derivative`**  
- `pos_dot`: Inertial velocity
- `vec_dot`: Inertial acceleration
- `q_dot`: Quaternion attitude derivative
- `omega_dot`: Body angular acceleration
- `mass_dot`: Mass rate

**`NavigationState`**  
- `pos_inertial`: Estimated inertial position
- `vec_inertial`: Estimated inertial velocity
- `q_BI`: Estimated quaternion attitude (body -> inertial rotation)
- `omega_body`: Estimated body angular velocity

**`GuidanceCommand`**  
- `q_cmd_BI`: Target quaternion attitude
- `omega_cmd_body`: Target body angular velocity
- `throttle_cmd`: Target throttle value

**`ControlCommand`**  
- `throttle_cmd`: Commanded throttle value
- `gimbal_pitch_cmd`: Commanded gimbal pitch value
- `gimbal_yaw_cmd`: Commanded gimbal yaw value
- `engine_enable_cmd`: Commanded engine on/off

## Dynamics Breakdown

Forces and moments are computed via a modular system:

- Gravity Model
- Aero Model
- Propulsion Model
- Loads Aggregator

The `LoadsAggregator` combines all contributions and ensures frame consistency.

## GNC Architecture

The GNC stack is structured into three layers:

- **Navigation**: provides state estimates (currently truth-based)
- **Guidance**: computes desired attitude and throttle
- **Control**: computes actuator commands using a quaternion PD controller

## Attitude Control Math

Attitude error is computed as:

`q_err = q_cmd * q_current^{-1}`

The vector part of `q_err` is used as the proportional error term.

Control law:

`u = Kp * e - Kd * ω`

Gimbal commands are mapped from the control output using a simplified allocation.
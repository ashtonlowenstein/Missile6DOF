//
// Created by Ashton Lowenstein on 3/25/26.
//

#include "../../include/core/Integrator.h"

State Integrator::eulerStep(
    const MissileDynamics &dynamics,
    double t,
    double dt,
    const State &state,
    const ControlCommand &cmd)
{
    const Derivative dx = dynamics.derivatives(t, state, cmd);

    State next_state = state;

    next_state.pos_inertial   += dx.pos_dot * dt;
    next_state.vel_inertial   += dx.vel_dot * dt;
    next_state.q_BI     += dx.q_dot * dt;
    next_state.omega_body += dx.omega_dot * dt;

    next_state.q_BI.normalize();

    return next_state;
}

State Integrator::rk4Step(
    const MissileDynamics &dynamics,
    double t,
    double dt,
    const State &state,
    const ControlCommand &cmd)
{
    const Derivative k1 = dynamics.derivatives(t, state, cmd);

    State s2 = addScaled(state, k1, 0.5 * dt);
    s2.q_BI.normalize();
    const Derivative k2 = dynamics.derivatives(t + 0.5 * dt, s2, cmd);

    State s3 = addScaled(state, k2, 0.5 * dt);
    s3.q_BI.normalize();
    const Derivative k3 = dynamics.derivatives(t + 0.5 * dt, s3, cmd);

    State s4 = addScaled(state, k3, dt);
    s4.q_BI.normalize();
    const Derivative k4 = dynamics.derivatives(t + dt, s4, cmd);

    State next_state = state;

    // next_state.pos_inertial   += (k1.pos_dot + k2.pos_dot * 2.0 + k3.pos_dot * 2.0 + k4.pos_dot) * dt * (1.0/6.0);
    // next_state.vel_inertial   += (k1.vel_dot + k2.vel_dot * 2.0 + k3.vel_dot * 2.0 + k4.vel_dot) * dt * (1.0/6.0);
    // next_state.q_BI     += (k1.q_dot + k2.q_dot * 2.0 + k3.q_dot * 2.0 + k4.q_dot) * dt * (1.0/6.0);
    // next_state.omega_body += (k1.omega_dot + k2.omega_dot * 2.0 + k3.omega_dot * 2.0 + k4.omega_dot) * dt * (1.0/6.0);

    Derivative rk4_update = addRK4(k1, k2, k3, k4);

    next_state.pos_inertial += rk4_update.pos_dot * dt;

    next_state.vel_inertial += rk4_update.vel_dot * dt;

    next_state.q_BI += rk4_update.q_dot * dt;

    next_state.omega_body += rk4_update.omega_dot * dt;

    next_state.mass += rk4_update.mass_dot * dt;

    next_state.actuators.throttle += rk4_update.actuator_dot.throttle_dot * dt;

    next_state.actuators.gimbal_pitch += rk4_update.actuator_dot.gimbal_pitch_dot * dt;

    next_state.actuators.gimbal_yaw += rk4_update.actuator_dot.gimbal_yaw_dot * dt;

    next_state.q_BI.normalize();

    return next_state;
}

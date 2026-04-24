//
// Created by Ashton Lowenstein on 3/24/26.
//

#include "../../include/core/State.h"

State addScaled(const State& s, const Derivative& d, const double h) {
    State out = s;
    out.pos_inertial += d.pos_dot * h;
    out.vel_inertial += d.vel_dot * h;
    out.q_BI += d.q_dot * h;
    out.omega_body += d.omega_dot * h;
    out.actuators.throttle += d.actuator_dot.throttle_dot * h;
    out.actuators.gimbal_pitch += d.actuator_dot.gimbal_pitch_dot * h;
    out.actuators.gimbal_yaw += d.actuator_dot.gimbal_yaw_dot * h;
    return out;
}

Derivative addRK4(const Derivative &k1, const Derivative &k2, const Derivative &k3, const Derivative &k4) {
    Derivative out;

    out.pos_dot = (k1.pos_dot + k2.pos_dot * 2.0 + k3.pos_dot * 2.0 + k4.pos_dot) * (1.0/6.0);

    out.vel_dot = (k1.vel_dot + k2.vel_dot * 2.0 + k3.vel_dot * 2.0 + k4.vel_dot) * (1.0/6.0);

    out.q_dot = (k1.q_dot + k2.q_dot * 2.0 + k3.q_dot * 2.0 + k4.q_dot) * (1.0/6.0);

    out.omega_dot = (k1.omega_dot + k2.omega_dot * 2.0 + k3.omega_dot * 2.0 + k4.omega_dot) * (1.0/6.0);

    out.mass_dot = (k1.mass_dot + k2.mass_dot * 2.0 + k3.mass_dot * 2.0 + k4.mass_dot) * (1.0/6.0);

    out.actuator_dot.throttle_dot = (k1.actuator_dot.throttle_dot + k2.actuator_dot.throttle_dot * 2.0
        + k3.actuator_dot.throttle_dot * 2.0 + k4.actuator_dot.throttle_dot) * (1.0/6.0);

    out.actuator_dot.gimbal_pitch_dot = (k1.actuator_dot.gimbal_pitch_dot + k2.actuator_dot.gimbal_pitch_dot * 2.0
        + k3.actuator_dot.gimbal_pitch_dot * 2.0 + k4.actuator_dot.gimbal_pitch_dot) * (1.0/6.0);

    out.actuator_dot.gimbal_yaw_dot = (k1.actuator_dot.gimbal_yaw_dot + k2.actuator_dot.gimbal_yaw_dot * 2.0
        + k3.actuator_dot.gimbal_yaw_dot * 2.0 + k4.actuator_dot.gimbal_yaw_dot) * (1.0/6.0);

    return out;
}
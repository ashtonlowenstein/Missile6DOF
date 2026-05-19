//
// Created by Ashton Lowenstein on 4/20/26.
//

#ifndef MISSILE6DOF_LOGRECORD_H
#define MISSILE6DOF_LOGRECORD_H
#include "../MathTypes/Quaternion.h"

struct LogRecord {
    double t{0.0};

    Vec3 pos_inertial{};
    Vec3 vel_inertial{};

    Quaternion q_bi{};
    Vec3 omega_body{};
    Vec3 omega_body_dot{};

    double tilt{0.0};

    double mass{0.0};
    double mass_est{0.0};

    Vec3 pos_inertial_estimate{};
    Vec3 vel_inertial_estimate{};
    Quaternion q_bi_estimate{};
    Vec3 omega_body_estimate{};

    Quaternion q_cmd{};
    double q_error_norm{0.0};

    // actuator
    double throttle_cmd{0.0};
    double throttle_actual{0.0};
    double gimbal_pitch_cmd{0.0};
    double gimbal_pitch_actual{0.0};
    double gimbal_yaw_cmd{0.0};
    double gimbal_yaw_actual{0.0};

    // sensor reading
    Vec3 accel_body{};
    Vec3 accel_body_bias{};

    // propulsion
    Vec3 thrust_force_body{};
    Vec3 thrust_moment_body{};
    double thrust{0.0};
    double mdot{0.0};

    // aero / flight condition
    double air_speed{0.0};
    double alpha{0.0};
    double beta{0.0};
    double mach{0.0};
    double qbar{0.0};

    // totals
    Vec3 total_force_body{};
    Vec3 total_moment_body{};
    Vec3 total_force_inertial{};

    // inertia (just diagonal for now)
    double Ixx{0.0}, Iyy{0.0}, Izz{0.0};
};

#endif //MISSILE6DOF_LOGRECORD_H
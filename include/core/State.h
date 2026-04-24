//
// Created by Ashton Lowenstein on 3/24/26.
//

#ifndef MISSILE6DOF_STATE_H
#define MISSILE6DOF_STATE_H

#include "../MathTypes/Mat3.h"
#include "../MathTypes/Quaternion.h"


struct PropulsionActuatorState {
    double throttle{0.0};     // actual realized throttle [0,1]
    double gimbal_pitch{0.0}; // rad
    double gimbal_yaw{0.0};   // rad
};

struct PropulsionActuatorDerivative {
    double throttle_dot{0.0};
    double gimbal_pitch_dot{0.0};
    double gimbal_yaw_dot{0.0};
};

struct State {
    Vec3 pos_inertial{}; // inertial position (m)
    Vec3 vel_inertial{}; // inertial velocity (m/s)
    Quaternion q_BI{}; // attitude: body -> inertial
    Vec3 omega_body{}; // body angular velocity (rad/s)
    double mass = 0.0; // dynamic mass (kg)
    PropulsionActuatorState actuators{}; // generalized actuator dynamic states
};

struct Derivative {
    Vec3 pos_dot{}; // inertial velocity (m/s)
    Vec3 vel_dot{}; // inertial acceleration (m/s^2)
    Quaternion q_dot{};
    Vec3 omega_dot{}; // body angular acceleration (rad/s^2)
    double mass_dot{0.0};
    PropulsionActuatorDerivative actuator_dot{};
};

struct NavigationState {
    Vec3 pos_inertial{};
    Vec3 vel_inertial{};
    Quaternion q_BI{};
    Vec3 omega_body{};
};

struct GuidanceCommand {
    Quaternion q_cmd_BI{};
    Vec3 omega_cmd_body{}; // rad/s
    double throttle_cmd{0.0}; // [0,1]
};

struct ControlCommand {
    double throttle_cmd{0.0}; // [0,1]
    double gimbal_pitch_cmd{0.0}; // rad
    double gimbal_yaw_cmd{0.0}; // rad
    bool engine_enable_cmd{false};
};

struct ActuatorOutput {
    double throttle{0.0};      // realized [0,1]
    double gimbal_pitch{0.0};  // realized rad
    double gimbal_yaw{0.0};    // realized rad
    bool engine_enable = false;   // realized enable state
};

struct SimpleActuatorParams {
    double throttle_time_constant;   // s
    double throttle_rate_limit;      // 1/s

    double gimbal_time_constant;     // s
    double gimbal_rate_limit;        // rad/s
    double gimbal_limit;             // rad
};

struct MassProperties {
    double mass = 0.0;
    Vec3 com_body;
    Mat3 inertia_body;
    Mat3 inertia_body_inverse;
};

struct GravityLoads {
    Vec3 force_inertial{};
    Vec3 force_body{};
    Vec3 moment_body{};
};

struct AeroLoads {
    Vec3 force_body{};
    Vec3 moment_body{};
};

struct PropulsionLoads {
    Vec3 force_body;
    Vec3 moment_body;
    double thrust{0.0};
    double mdot{0.0};
    bool engine_active = false;
};

struct AtmosphereState {
    double density{0.0};
    double pressure{0.0};
    double temperature{0.0};
    double speed_of_sound{0.0};
    Vec3 wind_inertial{};
};

struct TotalLoads {
    GravityLoads gravity{};
    // Vec3 gravity_force_inertial{};
    // Vec3 gravity_force_body{};

    AeroLoads aero{};
    // Vec3 aero_force_body{};
    // Vec3 aero_moment_body{};

    PropulsionLoads propulsion{};
    // Vec3 propulsion_force_body{};
    // Vec3 propulsion_moment_body{};

    Vec3 total_force_body{};
    Vec3 total_force_inertial{};
    Vec3 total_moment_body{};

    double mdot{0.0};
};

struct FlightCondition {
    Vec3 pos_inertial;
    Vec3 vel_inertial;
    Vec3 air_rel_vel_body;
    Vec3 omega_body;

    double air_speed{0.0};
    double alpha{0.0};
    double beta{0.0};
    double mach{0.0};
    double qbar{0.0};

    AtmosphereState atmosphere{};
};

struct DynamicsContext {
    MassProperties mass_properties;
    FlightCondition flight_condition;
    ActuatorOutput actuator_output;
    TotalLoads loads;
};



State addScaled(const State& s, const Derivative& d, double h);

Derivative addRK4(const Derivative& k1, const Derivative& k2, const Derivative& k3, const Derivative& k4);


#endif //MISSILE6DOF_STATE_H
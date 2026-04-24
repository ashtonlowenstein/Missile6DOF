//
// Created by Ashton Lowenstein on 3/25/26.
//

#include "../../include/core/Simulator.h"
#include "../../include/core/Integrator.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include "../../include/logging/CSVLogger.h"

Simulator::Simulator(
    const MissileDynamics &dynamics,
    std::unique_ptr<IGuidanceModel> guidance,
    std::unique_ptr<INavigationModel> navigation,
    std::unique_ptr<IControllerModel> controller,
    double dt,
    double t_end)
:   dynamics_(dynamics),
    guidance_(std::move(guidance)),
    navigation_(std::move(navigation)),
    controller_(std::move(controller)),
    dt_(dt),
    t_end_(t_end)
{
    if (dt_ <= 0.0) {
        throw std::invalid_argument("dt must be positive");
    }
    if (t_end_ < 0.0) {
        throw std::invalid_argument("t_end must be nonnegative");
    }
}

State Simulator::run(
    const State &initial_state)
const {
    State state = initial_state;
    double t = 0.0;

    while (t <= t_end_) {
        NavigationState nav = navigation_->estimate(t, state);

        GuidanceCommand guid = guidance_->compute(t, nav);

        ControlCommand cmd = controller_->compute(t, nav, guid);

        state = Integrator::rk4Step(dynamics_, t, dt_, state, cmd);
        t += dt_;
    }
    return state;
}

State Simulator::run(
    const State &initial_state,
    const std::string &output_path)
const {
    CSVLogger logger(output_path);

    State state = initial_state;
    double t = 0.0;

    while (t <= t_end_) {
        NavigationState nav = navigation_->estimate(t, state);

        GuidanceCommand guid = guidance_->compute(t, nav);

        ControlCommand cmd = controller_->compute(t, nav, guid);

        const DynamicsContext ctx = dynamics_.evaluate(t, state, cmd);
        const Derivative ds = dynamics_.derivatives(t, state, cmd);

        LogRecord rec = fillLogRecord(t, state, ds, cmd, ctx);

        logger.log(rec);

        state = Integrator::rk4Step(dynamics_, t, dt_, state, cmd);
        t += dt_;
    }
    return state;
}

LogRecord Simulator::fillLogRecord(
        double t,
        const State& state,
        const Derivative& ds,
        const ControlCommand& cmd,
        const DynamicsContext& ctx) const {
    LogRecord rec{};
    rec.t = t;

    rec.pos_inertial = state.pos_inertial;
    rec.vel_inertial = state.vel_inertial;
    rec.q_bi = state.q_BI;
    rec.omega_body = state.omega_body;
    rec.omega_body_dot = ds.omega_dot;
    rec.mass = state.mass;

    rec.throttle_cmd = cmd.throttle_cmd;
    rec.throttle_actual = ctx.actuator_output.throttle;

    rec.gimbal_pitch_cmd = cmd.gimbal_pitch_cmd;
    rec.gimbal_pitch_actual = ctx.actuator_output.gimbal_pitch;

    rec.gimbal_yaw_cmd = cmd.gimbal_yaw_cmd;
    rec.gimbal_yaw_actual = ctx.actuator_output.gimbal_yaw;

    rec.q_cmd = guidance_->compute(t, navigation_->estimate(t, state)).q_cmd_BI;
    rec.q_error_norm = norm(toVector(rec.q_cmd * state.q_BI.conjugate()));

    rec.thrust_force_body = ctx.loads.propulsion.force_body;
    rec.thrust_moment_body = ctx.loads.propulsion.moment_body;
    rec.thrust = ctx.loads.propulsion.thrust;
    rec.mdot = ctx.loads.propulsion.mdot;

    rec.air_speed = ctx.flight_condition.air_speed;
    rec.alpha = ctx.flight_condition.alpha;
    rec.beta = ctx.flight_condition.beta;
    rec.mach = ctx.flight_condition.mach;
    rec.qbar = ctx.flight_condition.qbar;

    rec.total_force_body = ctx.loads.total_force_body;
    rec.total_moment_body = ctx.loads.total_moment_body;

    rec.Ixx = ctx.mass_properties.inertia_body(0,0);
    rec.Iyy = ctx.mass_properties.inertia_body(1,1);
    rec.Izz = ctx.mass_properties.inertia_body(2,2);

    return rec;
}
//
// Created by Ashton Lowenstein on 3/24/26.
//

#include "../../include/dynamics/MissileDynamics.h"

#include <algorithm>
#include <cmath>
#include "../../include/ModelInterfaces/IAeroModel.h"
#include "../../include/ModelInterfaces/IThrustModel.h"

MissileDynamics::MissileDynamics(
    std::unique_ptr<IGravityModel> grav,
    std::unique_ptr<IAtmosphereModel> atmosphere_model,
    std::unique_ptr<IAeroModel> aero_model,
    std::unique_ptr<IPropulsionModel> propulsion_model,
    std::unique_ptr<IMassPropertiesModel> mp,
    std::unique_ptr<IActuatorModel> actuator_model)
    :
        gravity_(std::move(grav)),
        atmosphere_(std::move(atmosphere_model)),
        aero_(std::move(aero_model)),
        propulsion_(std::move(propulsion_model)),
        mass_model_(std::move(mp)),
        actuator_model_(std::move(actuator_model)),
        loads_aggregator_(*gravity_, *propulsion_, *aero_)
{}

Derivative MissileDynamics::derivatives(
    double t,
    const State &state,
    const ControlCommand& cmd
) const {

    Derivative derivatives;
    const DynamicsContext ctx = evaluate(t, state, cmd);
    // const MassProperties mp = mass_model_->compute(state);
    // const FlightCondition fc = makeFlightCondition(t, state);
    // const ActuatorOutput act = actuator_model_->output(state, cmd);

    const PropulsionActuatorDerivative act_dot = actuator_model_->derivatives(state, cmd);

    // const TotalLoads total_loads = loads_aggregator_.compute(t, state, fc, mp, act);

    derivatives.pos_dot = state.vel_inertial;
    derivatives.vel_dot = ctx.loads.total_force_inertial / ctx.mass_properties.mass;

    const Quaternion omega_q = toQuaternion(state.omega_body);
    derivatives.q_dot = state.q_BI * omega_q * 0.5;

    const Vec3 Iomega = ctx.mass_properties.inertia_body * state.omega_body;
    derivatives.omega_dot = ctx.mass_properties.inertia_body_inverse * (ctx.loads.total_moment_body - cross(state.omega_body, Iomega));

    derivatives.mass_dot = -ctx.loads.mdot;
    derivatives.actuator_dot = act_dot;

    return derivatives;
}

DynamicsContext MissileDynamics::evaluate(double t, const State &state, const ControlCommand &cmd) const {
    (void)t; // remove this if/when time is used directly

    DynamicsContext ctx{};

    ctx.mass_properties = mass_model_->compute(state);

    ctx.flight_condition = makeFlightCondition(t, state);

    ctx.actuator_output =
        actuator_model_->output(state, cmd);

    ctx.loads =
        loads_aggregator_.compute(
            t,
            state,
            ctx.flight_condition,
            ctx.mass_properties,
            ctx.actuator_output);

    return ctx;
}


// Derivative MissileDynamics::derivativesFromLoads(
//     const State &state,
//     const Vec3 &f_aero_body,
//     const Vec3 &f_thrust_body,
//     const Vec3 &moment_body) const
// {
//     // Inputs:
//     //   f_aero_body   : body-frame aerodynamic force
//     //   f_thrust_body : body-frame thrust force
//     //   moment_body   : body-frame total applied moment
//     //
//     // State:
//     //   vel   : inertial velocity
//     //   q     : body-to-inertial attitude quaternion
//     //   omega : body-frame angular velocity
//
//     // unpack the state
//     // const Vec3 pos = state.pos;
//     const Vec3 vel = state.vel;
//     const Quaternion q_B_to_I = state.q;
//     const Vec3 omega = state.omega;
//
//     // construct forces
//     const Vec3 f_aero_inertial = q_B_to_I.rotateVector(f_aero_body);
//     const Vec3 f_thrust_inertial = q_B_to_I.rotateVector(f_thrust_body);
//
//     // create translational derivatives
//     const Vec3 pos_dot = vel;
//     const Vec3 vel_dot = gravity_accel_ + (f_aero_inertial + f_thrust_inertial) / mass_;
//
//     //create rotational derivatives
//     const Vec3 Iomega = I_ * omega;
//     const Vec3 omega_dot = I_inv_ * (moment_body - cross(omega, Iomega));
//     const Quaternion omega_q = toQuaternion(omega);
//     const Quaternion q_dot = (q_B_to_I * omega_q) * 0.5;
//
//     return Derivative{pos_dot, vel_dot, q_dot, omega_dot};
// }

FlightCondition MissileDynamics::makeFlightCondition(double t, const State &state) const {
    (void) t;

    FlightCondition fc;
    fc.pos_inertial = state.pos_inertial;
    fc.vel_inertial = state.vel_inertial;
    fc.omega_body = state.omega_body;

    fc.atmosphere = atmosphere_->sample(state.pos_inertial);

    const Vec3 vel_rel_inertial = state.vel_inertial - fc.atmosphere.wind_inertial;

    const Vec3 vel_rel_body = state.q_BI.conjugate().rotateVector(vel_rel_inertial);
    fc.air_rel_vel_body = vel_rel_body;

    const double u = vel_rel_body.x;
    const double v = vel_rel_body.y;
    const double w = vel_rel_body.z;

    fc.air_speed = std::sqrt(u * u + v * v + w * w);

    if (fc.air_speed > 1e-8) {
        fc.alpha = std::atan2(w, u);
        const double sin_beta = std::clamp(v / fc.air_speed, -1.0, 1.0);
        fc.beta  = std::asin(sin_beta);
        fc.qbar  = 0.5 * fc.atmosphere.density * fc.air_speed * fc.air_speed;
        if (fc.atmosphere.speed_of_sound > 1e-8) {
            fc.mach = fc.air_speed / fc.atmosphere.speed_of_sound;
        } else {
            fc.mach = 0.0;
        }
    } else {
        fc.alpha = 0.0;
        fc.beta  = 0.0;
        fc.qbar  = 0.0;
        fc.mach  = 0.0;
    }

    return fc;
}

FlightCondition MissileDynamics::reportFlightCondition(double t, const State& state) const {
    return makeFlightCondition(t, state);
}

//  TotalLoads MissileDynamics::reportLoads() const {
//      return current_loads_;
// }
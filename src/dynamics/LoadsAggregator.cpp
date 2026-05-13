//
// Created by Ashton Lowenstein on 4/14/26.
//

#include "../../include/dynamics/LoadsAggregator.h"

LoadsAggregator::LoadsAggregator(
    const IGravityModel& gravity_model,
    const IPropulsionModel& propulsion_model,
    const IAeroModel& aero_model
) :
    gravity_model_(gravity_model),
    propulsion_model_(propulsion_model),
    aero_model_(aero_model)
{}

TotalLoads LoadsAggregator::compute(
    double t,
    const State &state,
    const FlightCondition &fc,
    const MassProperties &mp,
    const ActuatorOutput& act)
const {
    TotalLoads loads{};

    // Gravity
    loads.gravity.force_inertial = mp.mass * gravity_model_.compute(state.pos_inertial).accel_inertial;
    loads.gravity.force_body = state.q_BI.conjugate().rotateVector(loads.gravity.force_inertial);

    loads.total_force_body += loads.gravity.force_body;
    loads.total_moment_body += loads.gravity.moment_body;

    // Aero
    loads.aero = aero_model_.computeLoads(t, state, fc, act, mp);

    loads.total_force_body += loads.aero.force_body;
    loads.total_moment_body += loads.aero.moment_body;

    // Propulsion
    loads.propulsion = propulsion_model_.compute(state, fc, act, mp);

    loads.total_force_body += loads.propulsion.force_body;
    loads.total_moment_body += loads.propulsion.moment_body;

    loads.mdot += loads.propulsion.mdot;

    // update total inertial force
    loads.total_force_inertial = state.q_BI.rotateVector(loads.total_force_body);
    //loads.total_force_inertial += loads.gravity.force_inertial;

    return loads;
}

//
// Created by Ashton Lowenstein on 3/24/26.
//

#ifndef MISSILE6DOF_MISSILEDYNAMICS_H
#define MISSILE6DOF_MISSILEDYNAMICS_H
#include <memory>

#include "LoadsAggregator.h"
#include "../MathTypes/Mat3.h"
#include "../core/State.h"
#include "../ModelInterfaces/IActuatorModel.h"
#include "../ModelInterfaces/IAeroModel.h"
#include "../ModelInterfaces/IAtmosphere.h"
#include "../ModelInterfaces/IMassProperties.h"
#include "../ModelInterfaces/IThrustModel.h"

struct LoadSummary {
    AeroLoads aero;
    ThrustLoads thrust;
    Vec3 total_force_body;
    Vec3 total_moment_body;
    Vec3 total_force_inertial;
    FlightCondition fc;
};

class MissileDynamics {
public:
    MissileDynamics(
        std::unique_ptr<IGravityModel> grav,
        std::unique_ptr<IAtmosphereModel> atmosphere_model,
        std::unique_ptr<IAeroModel> aero_model,
        std::unique_ptr<IPropulsionModel> propulsion_model,
        std::unique_ptr<IMassPropertiesModel> mp,
        std::unique_ptr<IActuatorModel> actuator_model);

    [[nodiscard]] Derivative derivatives(
        double t,
        const State& state,
        const ControlCommand& cmd
    ) const;

    // [[nodiscard]] Derivative derivativesFromLoads(
    //     const State& state,
    //     const Vec3& f_aero_body,
    //     const Vec3& f_thrust_body,
    //     const Vec3& moment_body
    // ) const;

    [[nodiscard]] FlightCondition reportFlightCondition(double t, const State& state) const;

    // [[nodiscard]] TotalLoads reportLoads() const;

    [[nodiscard]] DynamicsContext evaluate(double t, const State& state, const ControlCommand& cmd) const;

private:
    [[nodiscard]] FlightCondition makeFlightCondition(double t, const State& state) const;

    std::unique_ptr<IGravityModel> gravity_;
    std::unique_ptr<IAtmosphereModel> atmosphere_;
    std::unique_ptr<IAeroModel> aero_;
    std::unique_ptr<IPropulsionModel> propulsion_;
    std::unique_ptr<IMassPropertiesModel> mass_model_;
    std::unique_ptr<IActuatorModel> actuator_model_;
    LoadsAggregator loads_aggregator_;

    // Mat3 I_, I_inv_;
    // double mass_;
    // const Vec3 gravity_accel_;
};


#endif //MISSILE6DOF_MISSILEDYNAMICS_H
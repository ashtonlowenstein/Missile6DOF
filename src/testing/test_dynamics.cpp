//
// Created by Ashton Lowenstein on 4/24/26.
//

#include "../include/dynamics/MissileDynamics.h"
#include "test_utils.h"
#include "ModelImplementations/ConstantAtmosphere.h"
#include "ModelImplementations/SimpleActuatorModel.h"
#include "ModelImplementations/SimpleRocketMassPropertiesModel.h"
#include "ModelImplementations/ZeroAeroModel.h"
#include "ModelImplementations/ZeroGravityModel.h"
#include "ModelImplementations/ZeroPropulsionModel.h"

int main() {
    // Build zero-aero, zero-gravity, zero-propulsion or straight-through-COM dynamics here.
    auto aero_model = std::make_unique<ZeroAeroModel>();
    auto gravity_model = std::make_unique<ZeroGravityModel>();
    auto propulsion_model = std::make_unique<ZeroPropulsionModel>();
    std::unique_ptr<IAtmosphereModel>atmosphere_model =
        std::make_unique<ConstantAtmosphereModel>(
            1.0, 1.0, 273.0, 343.0, Vec3{0.0, 0.0, 0.0});
    SimpleRocketMassPropertiesModel mass_properties(
        30.0,
        50.0,
        Mat3::diagonal(5.0, 12.0, 12.0),
        Mat3::diagonal(8.0, 20.0, 20.0),
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0});
    auto mass_model = std::make_unique<SimpleRocketMassPropertiesModel>(mass_properties);
    SimpleActuatorParams actuator_params{
        .throttle_time_constant = 0.2,
        .throttle_rate_limit    = 10.0,
        .gimbal_time_constant   = 0.1,
        .gimbal_rate_limit      = 2.0,
        .gimbal_limit           = 10.0 * M_PI / 180.0
    };
    auto actuator_model = std::make_unique<SimpleActuatorModel>(actuator_params);
    MissileDynamics dynamics(
        std::move(gravity_model),
        std::move(atmosphere_model),
        std::move(aero_model),
        std::move(propulsion_model),
        std::move(mass_model),
        std::move(actuator_model)
        );

    State s{};
    s.q_BI = Quaternion{1,0,0,0};
    s.omega_body = Vec3{0,0,0};
    s.mass = 50.0;

    ControlCommand cmd{};
    cmd.throttle_cmd = 0.0;
    cmd.engine_enable_cmd = false;

    Derivative d = dynamics.derivatives(0.0, s, cmd);

    expectVecNear(d.omega_dot, Vec3{0,0,0}, 1e-12, "zero moment omega_dot");
    expectNear(d.q_dot.w, 0.0, 1e-12, "zero omega qdot w");
    expectNear(d.q_dot.x, 0.0, 1e-12, "zero omega qdot x");
    expectNear(d.q_dot.y, 0.0, 1e-12, "zero omega qdot y");
    expectNear(d.q_dot.z, 0.0, 1e-12, "zero omega qdot z");

    std::cout << "test_dynamics passed\n";
    return 0;
}

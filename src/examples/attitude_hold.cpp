//
// Created by Ashton Lowenstein on 4/24/26.
//

#include <iostream>

#include "../include/ModelImplementations/ConstantAtmosphere.h"
#include "../include/ModelImplementations/ZeroAeroModel.h"
#include "../include/MathTypes/Vec3.h"
#include "../include/core/State.h"
#include "../include/MathTypes/Mat3.h"
#include "../include/dynamics/MissileDynamics.h"
#include "../include/core/Simulator.h"
#include "../include/ModelImplementations/SimpleActuatorModel.h"
#include "../include/ModelImplementations/SimpleGimbaledPropulsionModel.h"
#include "../include/ModelImplementations/SimpleRocketMassPropertiesModel.h"
#include "../include/ModelImplementations/ZeroGravityModel.h"
#include "../include/GNCImplementations/GuidanceAttitudeHold.h"
#include "../include/GNCImplementations/NavigationTruthModel.h"
#include "../include/GNCImplementations/PDAttitudeController.h"

int main() {

    State s0{};

    s0.pos_inertial = Vec3{0.0, 0.0, 0.0};
    s0.vel_inertial = Vec3{0.0, 0.0, 0.0};

    s0.q_BI = Quaternion{Vec3{0.0, 1.0, 0.0}, 5.0 * M_PI / 180.0}
        * Quaternion{Vec3{0.0, 0.0, 1.0}, 5.0 * M_PI / 180.0};
    s0.omega_body = Vec3{0.0, 0.0, 0.0};

    s0.mass = 50.0;  // kg, example wet-ish mass

    s0.actuators.throttle = 0.0;
    s0.actuators.gimbal_pitch = 0.0;
    s0.actuators.gimbal_yaw = 0.0;


    // GNC Stack
    const Quaternion q_cmd_BI = Quaternion{1.0, 0.0, 0.0, 0.0};
    auto guidance = std::make_unique<GuidanceAttitudeHold>(q_cmd_BI, 1.0);

    auto navigation = std::make_unique<NavigationTruthModel>();

    constexpr double kp = 1.5;
    constexpr double kd = 0.15;
    auto controller = std::make_unique<PDAttitudeController>(kp, kd);

    //Propulsion
    auto propulsion_model = std::make_unique<SimpleGimbaledPropulsionModel>(
        100.0,
        200.0,
        Vec3{-1.0, 0.0, 0.0},
        30.0
    );

    //Mass
    SimpleRocketMassPropertiesModel mass_properties(
        30.0,
        50.0,
        Mat3::diagonal(5.0, 12.0, 12.0),
        Mat3::diagonal(8.0, 20.0, 20.0),
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0});
    auto mass_model = std::make_unique<SimpleRocketMassPropertiesModel>(mass_properties);

    // Gravity

    auto gravity_model = std::make_unique<ZeroGravityModel>();


    //Aero

    auto aero_model = std::make_unique<ZeroAeroModel>();


    std::unique_ptr<IAtmosphereModel>atmosphere_model =
        std::make_unique<ConstantAtmosphereModel>(
            1.0, 1.0, 273.0, 343.0, Vec3{0.0, 0.0, 0.0});

    // Actuators
    SimpleActuatorParams actuator_params{
        .throttle_time_constant = 0.2,
        .throttle_rate_limit    = 10.0,
        .gimbal_time_constant   = 0.1,
        .gimbal_rate_limit      = 2.0,
        .gimbal_limit           = 10.0 * M_PI / 180.0
    };

    auto actuator_model = std::make_unique<SimpleActuatorModel>(actuator_params);

    // Dynamics
    MissileDynamics dynamics(
        std::move(gravity_model),
        std::move(atmosphere_model),
        std::move(aero_model),
        std::move(propulsion_model),
        std::move(mass_model),
        std::move(actuator_model)
        );

    const Simulator sim(
        dynamics,
        std::move(guidance),
        std::move(navigation),
        std::move(controller),
        0.01,
        100.0);

    try {
        State final_State = sim.run(s0, "/Users/ashtonlowenstein/CLionProjects/Missile6DOF/new_test.csv");
        std::cout << "Simulation complete. Output written to output/sim.csv\n";
    } catch (const std::exception& e) {
        std::cerr << "Simulation failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

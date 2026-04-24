//
// Created by Ashton Lowenstein on 4/24/26.
//

#include <iostream>

#include "test_utils.h"
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

double attitudeErrorNorm(const Quaternion& q_current_BI,
                         const Quaternion& q_cmd_BI)
{
    Quaternion q_err = q_cmd_BI * q_current_BI.conjugate();

    if (q_err.w < 0.0) {
        q_err = q_err * -1.0;
    }

    return norm(Vec3{q_err.x, q_err.y, q_err.z});
}

int main() {

    constexpr double deg = M_PI / 180.0;

    // Yaw test
    State s0yaw_test{};

    s0yaw_test.pos_inertial = Vec3{0.0, 0.0, 0.0};
    s0yaw_test.vel_inertial = Vec3{0.0, 0.0, 0.0};

    s0yaw_test.q_BI = Quaternion{Vec3{0.0, 0.0, 1.0}, 5.0 * M_PI / 180.0};
    s0yaw_test.omega_body = Vec3{0.0, 0.0, 0.0};

    s0yaw_test.mass = 50.0;  // kg, example wet-ish mass

    s0yaw_test.actuators.throttle = 0.0;
    s0yaw_test.actuators.gimbal_pitch = 0.0;
    s0yaw_test.actuators.gimbal_yaw = 0.0;

    // Pitch test
    State s0pitch_test{};

    s0pitch_test.pos_inertial = Vec3{0.0, 0.0, 0.0};
    s0pitch_test.vel_inertial = Vec3{0.0, 0.0, 0.0};
    s0pitch_test.q_BI = Quaternion{Vec3{0.0, 1.0, 0.0}, 5.0 * M_PI / 180.0};
    s0pitch_test.omega_body = Vec3{0.0, 0.0, 0.0};
    s0pitch_test.mass = 50.0;  // kg, example wet-ish mass

    s0yaw_test.actuators.throttle = 0.0;
    s0yaw_test.actuators.gimbal_pitch = 0.0;
    s0yaw_test.actuators.gimbal_yaw = 0.0;

    // Pitch + Yaw test
    State s0both_test{};

    s0both_test.pos_inertial = Vec3{0.0, 0.0, 0.0};
    s0both_test.vel_inertial = Vec3{0.0, 0.0, 0.0};

    s0both_test.q_BI = Quaternion{Vec3{0.0, 1.0, 0.0}, 5.0 * M_PI / 180.0}
        * Quaternion{Vec3{0.0, 0.0, 1.0}, 5.0 * M_PI / 180.0};
    s0both_test.omega_body = Vec3{0.0, 0.0, 0.0};

    s0both_test.mass = 50.0;  // kg, example wet-ish mass

    s0both_test.actuators.throttle = 0.0;
    s0both_test.actuators.gimbal_pitch = 0.0;
    s0both_test.actuators.gimbal_yaw = 0.0;


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

    const double initial_error_pitch = norm(toVector(q_cmd_BI * s0pitch_test.q_BI.conjugate()));
    const double initial_error_yaw = norm(toVector(q_cmd_BI * s0yaw_test.q_BI.conjugate()));
    const double initial_error_both = norm(toVector(q_cmd_BI * s0both_test.q_BI.conjugate()));

    const State sf_pitch = sim.run(s0pitch_test);
    const State sf_yaw = sim.run(s0yaw_test);
    const State sf_both = sim.run(s0both_test);

    const double final_error_pitch = norm(toVector(q_cmd_BI * sf_pitch.q_BI.conjugate()));
    const double final_error_yaw = norm(toVector(q_cmd_BI * sf_yaw.q_BI.conjugate()));
    const double final_error_both = norm(toVector(q_cmd_BI * sf_both.q_BI.conjugate()));

    expectTrue(final_error_pitch < initial_error_pitch,
               "GNC attitude hold should reduce attitude error");

    expectTrue(final_error_pitch < 0.02,
               "GNC attitude hold final error should be small");

    expectTrue(final_error_yaw < initial_error_yaw,
               "GNC attitude hold should reduce attitude error");

    expectTrue(final_error_yaw < 0.02,
               "GNC attitude hold final error should be small");

    expectTrue(final_error_both < initial_error_both,
               "GNC attitude hold should reduce attitude error");

    expectTrue(final_error_both < 0.02,
               "GNC attitude hold final error should be small");

    std::cout << "test_gnc passed\n";

    return 0;
}
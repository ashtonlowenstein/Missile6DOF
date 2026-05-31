#include <iostream>
#include <sstream>
#include "../include/ModelImplementations/ConstantAtmosphere.h"
#include "../include/ModelImplementations/ZeroAeroModel.h"
#include "../include/MathTypes/Vec3.h"
#include "../include/core/State.h"
#include "../include/MathTypes/Mat3.h"
#include "../include/dynamics/MissileDynamics.h"
#include "../include/core/Simulator.h"
#include "../include/ModelImplementations/SimpleActuatorModel.h"
#include "../include/ModelImplementations/SimpleRocketMassPropertiesModel.h"
#include "../include/ModelImplementations/ConstantGravityModel.h"
#include "sensors/IdealImuModel.h"
#include "GNCImplementations/EKFNavigation.h"
#include "GNCImplementations/LandingGuidance.h"
#include "GNCImplementations/NavigationTruthModel.h"
#include "GNCImplementations/PDLandingController.h"
#include "MathTypes/MatrixXd.h"
#include "ModelImplementations/SimpleGimbaledPropulsionModel.h"
#include "sensors/IdealGpsModel.h"
#include "sensors/NoisyImuModel.h"

int main() {
    State s0{};

    s0.pos_inertial = Vec3{-1.0, 1.0, 100.0};
    s0.vel_inertial = Vec3{0.0, 0.0, -10.0};

    s0.q_BI = Quaternion{std::sqrt(0.5), 0.0, -std::sqrt(0.5), 0.0};
    s0.omega_body = Vec3{0.0, 0.0, 0.0};

    s0.mass = 10.0;  // kg, example wet-ish mass

    s0.actuators.throttle = 0.0;
    s0.actuators.gimbal_pitch = 0.0;
    s0.actuators.gimbal_yaw = 0.0;

    auto gravity = std::make_shared<ConstantGravityModel>();
    auto aero_model = std::make_unique<ZeroAeroModel>();
    std::unique_ptr<IAtmosphereModel>atmosphere_model =
        std::make_unique<ConstantAtmosphereModel>(
            1.0, 1.0, 273.0, 343.0, Vec3{0.0, 0.0, 0.0});

    SimpleActuatorParams actuator_params{
        .throttle_time_constant = 0.2,
        .throttle_rate_limit    = 10.0,
        .gimbal_time_constant   = 0.1,
        .gimbal_rate_limit      = 2.0,
        .gimbal_limit           = 10.0 * M_PI / 180.0
    };
    auto actuator_model = std::make_unique<SimpleActuatorModel>(actuator_params);

    SimpleRocketMassPropertiesModel mass_properties(
        5.0,
        5.0,
        Mat3::diagonal(5.0, 12.0, 12.0),
        Mat3::diagonal(8.0, 20.0, 20.0),
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0});
    auto mass_model = std::make_unique<SimpleRocketMassPropertiesModel>(mass_properties);

    //Propulsion
    double max_thrust = 250.0;
    double lever_arm = 1.0;
    auto propulsion_model = std::make_unique<SimpleGimbaledPropulsionModel>(
        max_thrust,
        200.0,
        Vec3{-lever_arm, 0.0, 0.0},
        5.0
    );

    auto sensors = std::make_unique<IdealImuModel>();

    const Vec3 target_pos{0.0, 0.0, 0.0};
    const Vec3 target_vel{0.0, 0.0, 0.0};
    const Quaternion target_q_BI{std::sqrt(0.5), 0.0, -std::sqrt(0.5), 0.0};
    auto guidance = std::make_unique<LandingGuidance>(
        target_pos,
        target_vel,
        target_q_BI
    );

    Mat<9,9> Q = zeros<9,9>();

    for (int i = 0; i < 3; ++i) {
        double sigma_bias_rw = 0.001;
        double sigma_accel_process = 0.05;
        Q(3+i, 3+i) = sigma_accel_process * sigma_accel_process * 0.01 * 0.01;
        Q(6+i, 6+i) = sigma_bias_rw * sigma_bias_rw * 0.01;
    }
    double sigma_gps = 0.5;
    auto gps_model = std::make_unique<IdealGpsModel>();
    Mat<3,3> R_gps = identity<3>() * (sigma_gps * sigma_gps);

    auto navigation = std::make_unique<EkfNavigation>(gravity, Q, R_gps, max_thrust, 0.2, 10.0);
    //auto navigation = std::make_unique<NavigationTruthModel>();

    MissileDynamics dynamics(
        std::move(gravity),
        std::move(atmosphere_model),
        std::move(aero_model),
        std::move(propulsion_model),
        std::move(mass_model),
        std::move(actuator_model)
    );

    LandingGains landing_gains{
        .kp_trans_xy = 0.5,
        .kp_trans_z = 0.6,
        .kd_trans_xy = 2.5,
        .kd_trans_z = 1.25,
        .kp_rot = 0.1,
        .kd_rot = 2.925
    };

    auto controller = std::make_unique<PDLandingController>(landing_gains, max_thrust, lever_arm, actuator_params);

    const Simulator sim(
        dynamics,
        std::move(sensors),
        std::move(gps_model),
        std::move(guidance),
        std::move(navigation),
        std::move(controller),
        0.01,
        10.0);

    try {
        // std::ostringstream oss;
        //
        // oss << std::fixed << std::setprecision(2)
        //     << "../view_"
        //     << "landing"
        //     << "_kpt_xy_" << kpt_xy
        //     << "_kpt_z_" << kpt_z
        //     << "_kdt_xy_" << kdt_xy
        //     << "_kdt_z_" << kdt_z
        //     << "_kpr_" << kpr
        //     << "_kdr_" << kdr
        //     << ".csv";

        //std::string filename = oss.str();
        std::string filename = "../landing_test.csv";
        State final_State = sim.run_gain_tuning(s0, filename);
        //std::cout << "Simulation complete. Output written to output/sim.csv\n";
    } catch (const std::exception& e) {
        std::cerr << "Simulation failed: " << e.what() << "\n";
        return 1;
    }
}
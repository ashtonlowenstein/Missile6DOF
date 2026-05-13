//
// Created by Ashton Lowenstein on 5/12/26.
//

#ifndef MISSILE6DOF_NOISYIMUMODEL_H
#define MISSILE6DOF_NOISYIMUMODEL_H

#include "IImuModel.h"
#include <random>

class NoisyImuModel : public IImuModel {
public:
    NoisyImuModel(
        const Vec3& accel_bias,
        const Vec3& gyro_bias,
        double sigma_accel,
        double sigma_gyro
    ) :
        accel_bias_(accel_bias),
        gyro_bias_(gyro_bias),
        rng_(std::random_device()()),
        accel_normal_(0, sigma_accel),
        gyro_normal_(0, sigma_gyro) {}

    [[nodiscard]] ImuMeasurement measure(
        const State& state,
        const DynamicsContext& ctx,
        double t
    ) const override {

        ImuMeasurement m{};

        const Vec3 accel_noise{accel_normal_(rng_), accel_normal_(rng_), accel_normal_(rng_)};
        const Vec3 gyro_noise{gyro_normal_(rng_), gyro_normal_(rng_), gyro_normal_(rng_)};

        m.gyro_body = state.omega_body + gyro_noise + gyro_bias_;

        const Vec3 total_accel_body =
            ctx.loads.total_force_body / ctx.mass_properties.mass;

        const Vec3 gravity_accel_body =
            ctx.loads.gravity.force_body / ctx.mass_properties.mass;

        m.accel_body =
            total_accel_body - gravity_accel_body + accel_noise + accel_bias_;

        return m;
    }

private:
    Vec3 accel_bias_;
    Vec3 gyro_bias_;
    mutable std::mt19937 rng_;
    mutable std::normal_distribution<double> accel_normal_;
    mutable std::normal_distribution<double> gyro_normal_;
};

#endif //MISSILE6DOF_NOISYIMUMODEL_H
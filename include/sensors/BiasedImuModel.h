//
// Created by Ashton Lowenstein on 5/7/26.
//

#ifndef MISSILE6DOF_BIASEDIMUMODEL_H
#define MISSILE6DOF_BIASEDIMUMODEL_H

#include "IImuModel.h"

class BiasedImuModel : public IImuModel {
public:
    BiasedImuModel(const Vec3& accel_bias, const Vec3& gyro_bias)
        : accel_bias_(accel_bias), gyro_bias_(gyro_bias){}

    [[nodiscard]] ImuMeasurement measure(
        const State& truth,
        const DynamicsContext& ctx,
        double) const override {

        ImuMeasurement m{};

        m.gyro_body = truth.omega_body + gyro_bias_;

        //specific force, excluding gravity
        const Vec3 total_accel_body =
            ctx.loads.total_force_body / ctx.mass_properties.mass;

        const Vec3 gravity_accel_body =
            ctx.loads.gravity.force_body / ctx.mass_properties.mass;

        m.accel_body =
            total_accel_body - gravity_accel_body + accel_bias_;

        return m;
    }

private:
    Vec3 accel_bias_;
    Vec3 gyro_bias_;
};

#endif //MISSILE6DOF_BIASEDIMUMODEL_H
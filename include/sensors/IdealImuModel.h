//
// Created by Ashton Lowenstein on 5/6/26.
//

#ifndef MISSILE6DOF_IDEALIMUMODEL_H
#define MISSILE6DOF_IDEALIMUMODEL_H

#include "IImuModel.h"

class IdealImuModel : public IImuModel {
public:
    [[nodiscard]] ImuMeasurement measure(
        const State& truth,
        const DynamicsContext& ctx,
        double) const override {

        ImuMeasurement m{};

        m.gyro_body = truth.omega_body;

        //specific force, excluding gravity
        const Vec3 total_accel_body =
            ctx.loads.total_force_body / ctx.mass_properties.mass;

        const Vec3 gravity_accel_body =
            ctx.loads.gravity.force_body / ctx.mass_properties.mass;

        m.accel_body =
            total_accel_body - gravity_accel_body;

        return m;
    }
};

#endif //MISSILE6DOF_IDEALIMUMODEL_H
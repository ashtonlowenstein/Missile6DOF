//
// Created by Ashton Lowenstein on 4/22/26.
//

#ifndef MISSILE6DOF_CONSTANTGRAVITYMODEL_H
#define MISSILE6DOF_CONSTANTGRAVITYMODEL_H

#include "../ModelInterfaces/IGravityModel.h"

class ConstantGravityModel : public IGravityModel {
public:
    [[nodiscard]] GravityOutput compute(const Vec3& pos_inertial) const override {
        (void) pos_inertial;
        return {Vec3{0.0, 0.0, -9.8}};
    }
};

#endif //MISSILE6DOF_CONSTANTGRAVITYMODEL_H
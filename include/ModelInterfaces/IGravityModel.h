//
// Created by Ashton Lowenstein on 4/14/26.
//

#ifndef MISSILE6DOF_GRAVITYMODEL_H
#define MISSILE6DOF_GRAVITYMODEL_H

#include "../core/State.h"

struct GravityOutput {
    Vec3 accel_inertial{};
};

class IGravityModel {
public:
    virtual ~IGravityModel() = default;

    [[nodiscard]] virtual GravityOutput compute(const State& state) const = 0; // returns acceleration
};

#endif //MISSILE6DOF_GRAVITYMODEL_H
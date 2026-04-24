//
// Created by Ashton Lowenstein on 4/17/26.
//

#ifndef MISSILE6DOF_ZEROGRAVITYMODEL_H
#define MISSILE6DOF_ZEROGRAVITYMODEL_H

#include "../ModelInterfaces/IGravityModel.h"

class ZeroGravityModel : public IGravityModel {
public:
    GravityOutput compute(const State& state) const override {
        GravityOutput output{};
        output.accel_inertial = {0.0, 0.0, 0.0};
        return output;
    }
};

#endif //MISSILE6DOF_ZEROGRAVITYMODEL_H
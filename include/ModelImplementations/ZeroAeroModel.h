//
// Created by Ashton Lowenstein on 3/25/26.
//

#ifndef MISSILE6DOF_ZEROAEROMODEL_H
#define MISSILE6DOF_ZEROAEROMODEL_H
#include "../ModelInterfaces/IAeroModel.h"

class ZeroAeroModel : public IAeroModel {
public:
    [[nodiscard]] AeroLoads computeLoads(
        double t,
        const State &state,
        const FlightCondition &fc,
        const ActuatorOutput& act,
        const MassProperties& mp) const override {
        (void) t; (void) state; (void) fc;
        return AeroLoads{};
    }
};

#endif //MISSILE6DOF_ZEROAEROMODEL_H
//
// Created by Ashton Lowenstein on 4/22/26.
//

#ifndef MISSILE6DOF_ZEROPROPULSIONMODEL_H
#define MISSILE6DOF_ZEROPROPULSIONMODEL_H

#include "../core/State.h"
#include "../ModelInterfaces/IPropulsionModel.h"

class ZeroPropulsionModel : public IPropulsionModel {
public:
    [[nodiscard]] PropulsionLoads compute(
        const State& state,
        const FlightCondition& fc,
        const ActuatorOutput& act,
        const MassProperties& mp
    ) const override {
        return PropulsionLoads{};
    }
};

#endif //MISSILE6DOF_ZEROPROPULSIONMODEL_H
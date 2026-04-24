//
// Created by Ashton Lowenstein on 4/14/26.
//

#ifndef MISSILE6DOF_PROPULSIONMODEL_H
#define MISSILE6DOF_PROPULSIONMODEL_H

#include "../core/State.h"

class IPropulsionModel {
public:
    virtual ~IPropulsionModel() = default;

    [[nodiscard]] virtual PropulsionLoads compute(
        const State& state,
        const FlightCondition& fc,
        const ActuatorOutput& act,
        const MassProperties& mp
    ) const = 0;
};

#endif //MISSILE6DOF_PROPULSIONMODEL_H
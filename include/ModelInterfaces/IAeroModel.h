//
// Created by Ashton Lowenstein on 3/24/26.
//

#ifndef MISSILE6DOF_AEROMODEL_H
#define MISSILE6DOF_AEROMODEL_H

#include "../core/State.h"



class IAeroModel {
public:
    virtual ~IAeroModel() = default;
    [[nodiscard]] virtual AeroLoads computeLoads(
        double t,
        const State& state,
        const FlightCondition& fc,
        const ActuatorOutput& act,
        const MassProperties& mp
    ) const = 0;
};


#endif //MISSILE6DOF_AEROMODEL_H
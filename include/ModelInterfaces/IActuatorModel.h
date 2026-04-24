//
// Created by Ashton Lowenstein on 4/14/26.
//

#ifndef MISSILE6DOF_ACTUATORMODEL_H
#define MISSILE6DOF_ACTUATORMODEL_H

#include "../core/State.h"

class IActuatorModel {
public:
    virtual ~IActuatorModel() = default;

    [[nodiscard]] virtual ActuatorOutput output(
        const State& state,
        const ControlCommand& cmd
    ) const = 0;

    [[nodiscard]] virtual PropulsionActuatorDerivative derivatives(
        const State& state,
        const ControlCommand& cmd
    ) const = 0;
};


#endif //MISSILE6DOF_ACTUATORMODEL_H
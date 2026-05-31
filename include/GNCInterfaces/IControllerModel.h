//
// Created by Ashton Lowenstein on 4/23/26.
//

#ifndef MISSILE6DOF_ICONTROLLERMODEL_H
#define MISSILE6DOF_ICONTROLLERMODEL_H

#include "../core/State.h"

class IControllerModel {
public:
    virtual ~IControllerModel() = default;

    [[nodiscard]] virtual ControlCommand compute(
        double t,
        double dt,
        const NavigationState& nav,
        const GuidanceCommand& guid,
        const State& truth,
        const ControlCommand& prev_cmd
        ) const = 0;
};

#endif //MISSILE6DOF_ICONTROLLERMODEL_H
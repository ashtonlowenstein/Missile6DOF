//
// Created by Ashton Lowenstein on 3/25/26.
//

#ifndef MISSILE6DOF_INTEGRATOR_H
#define MISSILE6DOF_INTEGRATOR_H

#include "../dynamics/MissileDynamics.h"
#include "State.h"

class Integrator {
public:
    static State eulerStep(
        const MissileDynamics& dynamics,
        double t,
        double dt,
        const State& state,
        const ControlCommand& cmd);

    static State rk4Step(
        const MissileDynamics& dynamics,
        double t,
        double dt,
        const State& state,
        const ControlCommand& cmd);
};


#endif //MISSILE6DOF_INTEGRATOR_H
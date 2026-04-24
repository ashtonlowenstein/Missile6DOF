//
// Created by Ashton Lowenstein on 3/24/26.
//

#ifndef MISSILE6DOF_THRUSTMODEL_H
#define MISSILE6DOF_THRUSTMODEL_H
#include "../core/State.h"
#include "../MathTypes/Vec3.h"

struct ThrustLoads {
    Vec3 force_body{};
    Vec3 moment_body{};
};

class IThrustModel {
public:
    virtual ~IThrustModel() = default;

    [[nodiscard]] virtual ThrustLoads computeLoads(
        double t, const State& state, const FlightCondition& fc) const = 0;
};


#endif //MISSILE6DOF_THRUSTMODEL_H
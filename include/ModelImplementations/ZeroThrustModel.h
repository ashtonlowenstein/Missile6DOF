//
// Created by Ashton Lowenstein on 3/25/26.
//

#ifndef MISSILE6DOF_ZEROTHRUSTMODEL_H
#define MISSILE6DOF_ZEROTHRUSTMODEL_H

#include "../ModelInterfaces/IThrustModel.h"

class ZeroThrustModel : public IThrustModel {
public:
    // [[nodiscard]] Vec3 bodyForce(double t, const State& state) const override {
    //     return {0.0, 0.0, 0.0};
    // }
    //
    // [[nodiscard]] Vec3 bodyMoment(double t, const State& state) const override {
    //     return {0.0, 0.0, 0.0};
    // }
    [[nodiscard]] ThrustLoads computeLoads(double t, const State& state, const FlightCondition& fc) const override {
        (void) t; (void) state; (void) fc;
        return ThrustLoads{};
    }

};

#endif //MISSILE6DOF_ZEROTHRUSTMODEL_H
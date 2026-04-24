//
// Created by Ashton Lowenstein on 3/25/26.
//

#ifndef MISSILE6DOF_CONSTTHRUSTMODEL_H
#define MISSILE6DOF_CONSTTHRUSTMODEL_H

#include "../ModelInterfaces/IThrustModel.h"

class ConstThrustModel : public IThrustModel {
public:
    explicit ConstThrustModel(double thrust_mag) : thrust_mag_(thrust_mag) {}
    [[nodiscard]] ThrustLoads computeLoads(double t, const State &state, const FlightCondition &fc) const override {
        (void) t; (void) state; (void) fc;
        ThrustLoads loads{};
        loads.force_body = {thrust_mag_, 0.0, 0.0};
        return loads;
    }
private:
    double thrust_mag_;
};

#endif //MISSILE6DOF_CONSTTHRUSTMODEL_H
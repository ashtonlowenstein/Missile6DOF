//
// Created by Ashton Lowenstein on 4/23/26.
//

#ifndef MISSILE6DOF_NAVIGATIONTRUTHMODEL_H
#define MISSILE6DOF_NAVIGATIONTRUTHMODEL_H

#include "../GNCInterfaces/INavigationModel.h"

class NavigationTruthModel : public INavigationModel {
public:
    [[nodiscard]] NavigationState estimate(double, const State& state) const override {
        return {
            state.pos_inertial,
            state.vel_inertial,
            state.q_BI,
            state.omega_body
        };
    }
};

#endif //MISSILE6DOF_NAVIGATIONTRUTHMODEL_H
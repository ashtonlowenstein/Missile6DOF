//
// Created by Ashton Lowenstein on 4/23/26.
//

#ifndef MISSILE6DOF_NAVIGATIONTRUTHMODEL_H
#define MISSILE6DOF_NAVIGATIONTRUTHMODEL_H

#include "../GNCInterfaces/INavigationModel.h"

class NavigationTruthModel : public INavigationModel{
public:
    [[nodiscard]] NavigationState initializeFromTruth(const State& truth) const override {
        return {truth.pos_inertial, truth.vel_inertial, truth.q_BI, truth.omega_body};
    }
    [[nodiscard]] NavigationState estimate(
        double t,
        double dt,
        const State& state,
        const NavigationState& prev,
        const ImuMeasurement& sensors,
        const std::optional<GpsMeasurement>& gps
    ) const override {
        (void)t; (void)dt; (void) prev; (void)sensors; (void)gps;
        return  {state.pos_inertial,
                    state.vel_inertial,
                    state.q_BI,
                    state.omega_body};
    }
};

#endif //MISSILE6DOF_NAVIGATIONTRUTHMODEL_H
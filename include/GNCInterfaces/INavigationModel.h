//
// Created by Ashton Lowenstein on 4/23/26.
//

#ifndef MISSILE6DOF_INAVIGATIONMODEL_H
#define MISSILE6DOF_INAVIGATIONMODEL_H

#include "../core/State.h"

class INavigationModel {
public:
    virtual ~INavigationModel() = default;

    [[nodiscard]] virtual NavigationState initializeFromTruth(const State& truth) const = 0;

    [[nodiscard]] virtual NavigationState estimate(
        double t,
        double dt,
        const State& state,
        const NavigationState& prev,
        const ImuMeasurement& sensors,
        const ControlCommand& last_cmd,
        const std::optional<GpsMeasurement>& gps
    ) const = 0;
};

#endif //MISSILE6DOF_INAVIGATIONMODEL_H
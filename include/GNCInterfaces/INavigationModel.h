//
// Created by Ashton Lowenstein on 4/23/26.
//

#ifndef MISSILE6DOF_INAVIGATIONMODEL_H
#define MISSILE6DOF_INAVIGATIONMODEL_H

#include "../core/State.h"

class INavigationModel {
public:
    virtual ~INavigationModel() = default;
    [[nodiscard]] virtual NavigationState estimate(double t, const State& state) const = 0;
};

#endif //MISSILE6DOF_INAVIGATIONMODEL_H
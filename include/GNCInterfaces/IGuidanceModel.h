//
// Created by Ashton Lowenstein on 4/23/26.
//

#ifndef MISSILE6DOF_IGUIDANCEMODEL_H
#define MISSILE6DOF_IGUIDANCEMODEL_H

#include "../core/State.h"

class IGuidanceModel {
public:
    virtual ~IGuidanceModel() = default;
    [[nodiscard]] virtual GuidanceCommand compute(double t, const NavigationState& nav) const = 0;
};

#endif //MISSILE6DOF_IGUIDANCEMODEL_H
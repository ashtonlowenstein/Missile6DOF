//
// Created by Ashton Lowenstein on 5/11/26.
//

#ifndef MISSILE6DOF_IGPSMODEL_H
#define MISSILE6DOF_IGPSMODEL_H

#include "core/State.h"

class IGpsModel {
public:
    virtual ~IGpsModel() = default;

    [[nodiscard]] virtual std::optional<GpsMeasurement> measure(
        double t,
        const State& truth
    ) const = 0;
};

#endif //MISSILE6DOF_IGPSMODEL_H
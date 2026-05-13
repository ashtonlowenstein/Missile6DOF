//
// Created by Ashton Lowenstein on 5/6/26.
//

#ifndef MISSILE6DOF_IIMUMODEL_H
#define MISSILE6DOF_IIMUMODEL_H

#include "../core/State.h"

class IImuModel {
public:
    virtual ~IImuModel() = default;

    [[nodiscard]] virtual ImuMeasurement measure(
        const State& truth,
        const DynamicsContext& ctx,
        double t
    ) const = 0;
};

#endif //MISSILE6DOF_IIMUMODEL_H
//
// Created by Ashton Lowenstein on 5/11/26.
//

#ifndef MISSILE6DOF_IDEALGPSMODEL_H
#define MISSILE6DOF_IDEALGPSMODEL_H

#include "IGpsModel.h"

class IdealGpsModel : public IGpsModel {
public:
    [[nodiscard]] std::optional<GpsMeasurement> measure(
            double,
            const State& truth
        ) const override {

        return GpsMeasurement{
            truth.pos_inertial,
            true
        };
    }
};

#endif //MISSILE6DOF_IDEALGPSMODEL_H
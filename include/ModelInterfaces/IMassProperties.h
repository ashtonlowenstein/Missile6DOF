//
// Created by Ashton Lowenstein on 4/13/26.
//

#ifndef MISSILE6DOF_MASSPROPERTIES_H
#define MISSILE6DOF_MASSPROPERTIES_H
#include "../MathTypes/Mat3.h"
#include "../MathTypes/Vec3.h"
#include "../core/State.h"

class IMassPropertiesModel {
public:
    virtual ~IMassPropertiesModel() = default;

    [[nodiscard]] virtual MassProperties compute(const State& state) const = 0; // computes the vehicles mass-related properties
};

#endif //MISSILE6DOF_MASSPROPERTIES_H
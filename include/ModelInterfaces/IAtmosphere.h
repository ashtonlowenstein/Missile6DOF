//
// Created by Ashton Lowenstein on 3/24/26.
//

#ifndef MISSILE6DOF_ATMOSPHERE_H
#define MISSILE6DOF_ATMOSPHERE_H
#include "../core/State.h"
#include "../MathTypes/Vec3.h"

class IAtmosphereModel {
public:
    virtual ~IAtmosphereModel() = default;
    [[nodiscard]] virtual AtmosphereState sample(const Vec3& pos_inertial) const = 0;
};


#endif //MISSILE6DOF_ATMOSPHERE_H
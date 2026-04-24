//
// Created by Ashton Lowenstein on 4/15/26.
//

#ifndef MISSILE6DOF_SIMPLEROCKETMASSPROPERTIESMODEL_H
#define MISSILE6DOF_SIMPLEROCKETMASSPROPERTIESMODEL_H

#include <algorithm>

#include "../ModelInterfaces/IMassProperties.h"

class SimpleRocketMassPropertiesModel : public IMassPropertiesModel {
public:
    SimpleRocketMassPropertiesModel(
        double dry_mass,
        double wet_mass,
        const Mat3& inertia_dry,
        const Mat3& inertia_wet,
        const Vec3& com_dry,
        const Vec3& com_wet)
            : dry_mass_(dry_mass),
              wet_mass_(wet_mass),
              inertia_dry_(inertia_dry),
              inertia_wet_(inertia_wet),
              com_dry_(com_dry),
              com_wet_(com_wet)
    {}

    [[nodiscard]] MassProperties compute(const State& state) const override {
        MassProperties mass_prop;
        mass_prop.mass = state.mass;

        const double prop0 = wet_mass_ - dry_mass_;
        const double prop_remaining = std::clamp(state.mass - dry_mass_, 0.0, prop0);
        const double f = (prop0 > 0.0) ? (prop_remaining / prop0) : 0.0;

        mass_prop.com_body = com_dry_ + f * (com_wet_ - com_dry_);
        mass_prop.inertia_body = inertia_dry_ + (inertia_wet_ - inertia_dry_) * f;
        mass_prop.inertia_body_inverse = inverse(mass_prop.inertia_body);

        return mass_prop;
    }
private:
    double dry_mass_;
    double wet_mass_;
    Mat3 inertia_dry_;
    Mat3 inertia_wet_;
    Vec3 com_dry_;
    Vec3 com_wet_;
};

#endif //MISSILE6DOF_SIMPLEROCKETMASSPROPERTIESMODEL_H
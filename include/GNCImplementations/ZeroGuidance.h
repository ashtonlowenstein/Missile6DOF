//
// Created by Ashton Lowenstein on 5/6/26.
//

#ifndef MISSILE6DOF_ZEROGUIDANCE_H
#define MISSILE6DOF_ZEROGUIDANCE_H

#include "../GNCInterfaces/IGuidanceModel.h"

class ZeroGuidance : public IGuidanceModel {
public:
    explicit ZeroGuidance(double throttle) : throttle_(throttle){}
    [[nodiscard]] GuidanceCommand compute(
        double t,
        const NavigationState& nav
    ) const override {

        (void)t; (void)nav;

        return {Quaternion{}, Vec3{}, throttle_};
    }

private:
    double throttle_;
};

#endif //MISSILE6DOF_ZEROGUIDANCE_H
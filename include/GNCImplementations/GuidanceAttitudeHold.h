//
// Created by Ashton Lowenstein on 4/23/26.
//

#ifndef MISSILE6DOF_GUIDANCEATTITUDEHOLD_H
#define MISSILE6DOF_GUIDANCEATTITUDEHOLD_H

#include "../GNCInterfaces/IGuidanceModel.h"

class GuidanceAttitudeHold : public IGuidanceModel {
public:
    GuidanceAttitudeHold(const Quaternion& q_cmd, double throttle)
        : q_cmd_(q_cmd), throttle_(throttle) {}

    [[nodiscard]] GuidanceCommand compute(double, const NavigationState&) const override {
        return {
            q_cmd_,
            Vec3{0.0, 0.0, 0.0},
            throttle_
        };
    }
private:
    Quaternion q_cmd_;
    double throttle_;
};

#endif //MISSILE6DOF_GUIDANCEATTITUDEHOLD_H
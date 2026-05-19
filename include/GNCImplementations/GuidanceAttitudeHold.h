//
// Created by Ashton Lowenstein on 4/23/26.
//

#ifndef MISSILE6DOF_GUIDANCEATTITUDEHOLD_H
#define MISSILE6DOF_GUIDANCEATTITUDEHOLD_H

#include "../GNCInterfaces/IGuidanceModel.h"

class GuidanceAttitudeHold : public IGuidanceModel {
public:
    GuidanceAttitudeHold(const Vec3& accel_cmd_inertial, const Quaternion& q_cmd)
        : accel_cmd_(accel_cmd_inertial), q_cmd_(q_cmd) {}

    [[nodiscard]] GuidanceCommand compute(double, const NavigationState&) const override {
        return {
            accel_cmd_,
            Vec3{0.0, 0.0, 0.0},
            Vec3{0.0, 0.0, 0.0},
            q_cmd_,
            Vec3{0.0, 0.0, 0.0},
        };
    }
private:
    Vec3 accel_cmd_;
    Quaternion q_cmd_;
};

#endif //MISSILE6DOF_GUIDANCEATTITUDEHOLD_H
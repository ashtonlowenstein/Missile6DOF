//
// Created by Ashton Lowenstein on 5/14/26.
//

#ifndef MISSILE6DOF_LANDINGGUIDANCE_H
#define MISSILE6DOF_LANDINGGUIDANCE_H

#include "GNCInterfaces/IGuidanceModel.h"

class LandingGuidance : public IGuidanceModel {
public:
    LandingGuidance(const Vec3& pos_target_inertial, const Vec3& vel_target_inertial, const Quaternion& q_target_BI)
        : pos_target_{pos_target_inertial}, vel_target_(vel_target_inertial), q_target_(q_target_BI) {}

    [[nodiscard]] GuidanceCommand compute(double, const NavigationState&) const override {
        return {
            0.0,
            Vec3{0.0, 0.0, 0.0},
            pos_target_,
            vel_target_,
            q_target_,
            Vec3{0.0, 0.0, 0.0}
        };
    }

private:
    Vec3 pos_target_;
    Vec3 vel_target_;
    Quaternion q_target_;
};

#endif //MISSILE6DOF_LANDINGGUIDANCE_H
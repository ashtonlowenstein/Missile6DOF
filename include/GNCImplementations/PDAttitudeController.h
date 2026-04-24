//
// Created by Ashton Lowenstein on 4/23/26.
//

#ifndef MISSILE6DOF_PDATTITUDECONTROLLER_H
#define MISSILE6DOF_PDATTITUDECONTROLLER_H

#include "../GNCInterfaces/IControllerModel.h"

class PDAttitudeController : public IControllerModel {
public:
    PDAttitudeController(double kp, double kd) :
        kp_(kp),
        kd_(kd)
    {}

    [[nodiscard]] ControlCommand compute(
        double,
        const NavigationState& nav,
        const GuidanceCommand& guid)
    const override {
        const Quaternion& q = nav.q_BI;
        const Quaternion& q_cmd = guid.q_cmd_BI;

        Quaternion q_err = q_cmd * q.conjugate();

        if (q_err.w < 0.0) {
            q_err = q_err * -1.0;
        }

        const Vec3 e{q_err.x, q_err.y, q_err.z};
        const Vec3 omega = nav.omega_body;

        // PD torque-like command
        const Vec3 u = (kp_) * e - (kd_) * omega;

        // Map to gimbal commands (simple approximation)
        const double gimbal_pitch_cmd = -u.y;
        const double gimbal_yaw_cmd   = -u.z;

        return {
            guid.throttle_cmd,
            gimbal_pitch_cmd,
            gimbal_yaw_cmd,
            true
        };
    };

private:
    double kp_;
    double kd_;
};

#endif //MISSILE6DOF_PDATTITUDECONTROLLER_H
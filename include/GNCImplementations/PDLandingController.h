//
// Created by Ashton Lowenstein on 5/14/26.
//

#ifndef MISSILE6DOF_PDLANDINGCONTROLLER_H
#define MISSILE6DOF_PDLANDINGCONTROLLER_H

#include "GNCInterfaces/IControllerModel.h"
#include "MathTypes/Quaternion.h"
#include "MathTypes/RotationConversions.h"

class PDLandingController : public IControllerModel {
public:
    PDLandingController(
        double kp_trans_xy,
        double kp_trans_z,
        double kd_trans_xy,
        double kd_trans_z,
        double kp_rot,
        double kd_rot,
        double max_thrust) :
        kp_trans_xy_(kp_trans_xy),
        kp_trans_z_(kp_trans_z),
        kd_trans_xy_(kd_trans_xy),
        kd_trans_z_(kd_trans_z),
        kp_rot_(kp_rot),
        kd_rot_(kd_rot),
        max_thrust_(max_thrust) {}

    [[nodiscard]] ControlCommand compute(
        double,
        const NavigationState& nav,
        const GuidanceCommand& guid
    ) const override {

        // double z_error = nav.pos_inertial.z - guid.pos_cmd_inertial.z;
        // double vz_error = nav.vel_inertial.z - guid.vel_cmd_inertial.z;
        //
        // double accel_cmd_z = -kp_ * z_error - kd_ * vz_error + g0_;
        //
        // double thrust_req = nav.mass_est * accel_cmd_z;

        const Vec3 pos_error_I = guid.pos_cmd_inertial - nav.pos_inertial;
        // const Vec3 pos_error_I_xy = {pos_error_I.x, pos_error_I.y, 0.0};
        // const Vec3 pos_error_I_z = {0.0, 0.0, pos_error_I.z};

        const Vec3 vel_error_I = guid.vel_cmd_inertial - nav.vel_inertial;
        // const Vec3 vel_error_I_xy = {vel_error_I.x, vel_error_I.y, 0.0};
        // const Vec3 vel_error_I_z = {0.0, 0.0, vel_error_I.z};

        // const Vec3 a_cmd_I = kp_trans_xy_ * pos_error_I_xy + kp_trans_z_ * pos_error_I_z +
        //     kd_trans_xy_ * vel_error_I_xy + kd_trans_z_ * vel_error_I_z;

        const Vec3 a_cmd_I{
            kp_trans_xy_ * pos_error_I.x + kd_trans_xy_ * vel_error_I.x,
            kp_trans_xy_ * pos_error_I.y + kd_trans_xy_ * vel_error_I.y,
            kp_trans_z_  * pos_error_I.z + kd_trans_z_  * vel_error_I.z
        };

        const Vec3 a_thrust_cmd_I = a_cmd_I - g0_I_;

        Vec3 a_limited_I = a_thrust_cmd_I;

        const Vec3 a_h_I{a_limited_I.x, a_limited_I.y, 0.0};
        const double a_h_norm = norm(a_h_I);

        const double a_z = std::max(a_limited_I.z, 1e-6);
        const double a_h_max = std::tan(max_tilt_rad_) * a_z;

        if (a_h_norm > a_h_max) {
            const Vec3 a_h_limited_I = a_h_I * (a_h_max / a_h_norm);
            a_limited_I.x = a_h_limited_I.x;
            a_limited_I.y = a_h_limited_I.y;
        }

        const double a_thrust_norm = norm(a_limited_I);
        const Vec3 xI = a_limited_I / a_thrust_norm;

        const double thrust_req = nav.mass_est * a_thrust_norm;

        Vec3 y_ref = {0.0, 1.0, 0.0};
        Vec3 yI = y_ref - dot(xI, y_ref) * xI;
        if (norm(yI) < 1e-8) {
            y_ref = Vec3{0.0, 0.0, 1.0};
            yI = y_ref - dot(y_ref, xI) * xI;
        }
        yI = yI / norm(yI);
        const Vec3 zI = cross(xI, yI);

        Mat<3,3> R_cmd_BI{};
        R_cmd_BI(0,0) = xI.x; R_cmd_BI(0, 1) = yI.x; R_cmd_BI(0,2) = zI.x;
        R_cmd_BI(1,0) = xI.y; R_cmd_BI(1,1) = yI.y; R_cmd_BI(1,2) = zI.y;
        R_cmd_BI(2,0) = xI.z; R_cmd_BI(2,1) = yI.z; R_cmd_BI(2,2) = zI.z;

        Quaternion q_cmd_BI = rotationToQuaternion(R_cmd_BI);
        q_cmd_BI.normalize();

        if (dot(q_cmd_BI, nav.q_BI) < 0.0) {
            q_cmd_BI = q_cmd_BI * -1.0;
        }

        const Quaternion q_err = q_cmd_BI * nav.q_BI.conjugate();
        const Vec3 e{q_err.x, q_err.y, q_err.z};

        const Vec3 u = kp_rot_ * e - kd_rot_ * nav.omega_body;

        const double gimbal_pitch_cmd = -u.y;
        const double gimbal_yaw_cmd   = -u.z;


        ControlCommand cmd{};

        cmd.throttle_cmd = std::clamp(thrust_req / max_thrust_, 0.0, 1.0);
        cmd.gimbal_pitch_cmd = gimbal_pitch_cmd;
        cmd.gimbal_yaw_cmd = gimbal_yaw_cmd;
        cmd.engine_enable_cmd = true;

        return cmd;
    }

private:
    double kp_trans_xy_;
    double kp_trans_z_;
    double kd_trans_xy_;
    double kd_trans_z_;
    double kp_rot_;
    double kd_rot_;
    double max_thrust_;
    Vec3 g0_I_ = {0.0, 0.0, -9.81};
    double max_tilt_rad_ = 10 * M_PI / 180.0;
};

#endif //MISSILE6DOF_PDLANDINGCONTROLLER_H
//
// Created by Ashton Lowenstein on 5/14/26.
//

#ifndef MISSILE6DOF_PDLANDINGCONTROLLER_H
#define MISSILE6DOF_PDLANDINGCONTROLLER_H

#include "GNCInterfaces/IControllerModel.h"
#include "MathTypes/Quaternion.h"
#include "MathTypes/RotationConversions.h"
#include "core/State.h"

struct LandingGains {
    double kp_trans_xy{0.0};
    double kp_trans_z{0.0};
    double kd_trans_xy{0.0};
    double kd_trans_z{0.0};

    double kp_rot{0.0};
    double kd_rot{0.0};
};

inline double limitRate(double cmd, double prev, double max_rate, double dt) {
    const double delta = cmd - prev;
    const double max_delta = max_rate * dt;
    return prev + std::clamp(delta, -max_delta, max_delta);
}

class PDLandingController : public IControllerModel {
public:
    PDLandingController(
        const LandingGains& gains,
        double max_thrust,
        double lever_arm,
        const SimpleActuatorParams& actuator_params) :
        K_(gains),
        max_thrust_(max_thrust),
        lever_arm_(lever_arm),
        params_(actuator_params){}

    [[nodiscard]] ControlCommand compute(
        double,
        double dt,
        const NavigationState& nav,
        const GuidanceCommand& guid,
        const State& truth,
        const ControlCommand& prev_cmd
    ) const override {
        ControlCommand cmd{};

        Vec3 pos_error_I = guid.pos_cmd_inertial - nav.pos_inertial;
        if (const Vec3 pos_error_I_xy = {pos_error_I.x, pos_error_I.y, 0.0}; norm(pos_error_I_xy) < 1e-2) {
            pos_error_I.x = 0.0;
            pos_error_I.y = 0.0;
        }

        Vec3 vel_error_I = guid.vel_cmd_inertial - nav.vel_inertial;
        if (const Vec3 vel_error_I_xy = {vel_error_I.x, vel_error_I.y, 0.0}; norm(vel_error_I_xy) < 1e-2) {
            vel_error_I.x = 0.0;
            vel_error_I.y = 0.0;
        }

        const Vec3 a_cmd_I{
            K_.kp_trans_xy * pos_error_I.x + K_.kd_trans_xy * vel_error_I.x,
            K_.kp_trans_xy * pos_error_I.y + K_.kd_trans_xy * vel_error_I.y,
            K_.kp_trans_z  * pos_error_I.z + K_.kd_trans_z  * vel_error_I.z
        };

        const Vec3 a_thrust_cmd_I = a_cmd_I - g0_I_;

        Vec3 a_limited_I = a_thrust_cmd_I;
        a_limited_I.z = std::max(a_limited_I.z, 1.0);

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
        // raw desired thrust direction
        const Vec3 xI_raw = a_limited_I / a_thrust_norm;

        // filter desired thrust axis
        if (!initialized_) {
            xI_cmd = nav.q_BI.rotateVector(Vec3{1.0, 0.0, 0.0});
            xI_cmd = xI_cmd / norm(xI_cmd);
            initialized_ = true;
        }
        xI_cmd = alpha_ * xI_cmd + (1.0 - alpha_) * xI_raw;
        xI_cmd = xI_cmd / norm(xI_cmd);

        const double thrust_req = std::clamp(nav.mass_est * a_thrust_norm, 0.0, max_thrust_);

        Vec3 y_ref = {0.0, 1.0, 0.0};
        Vec3 yI = y_ref - dot(xI_cmd, y_ref) * xI_cmd;
        if (norm(yI) < 1e-8) {
            y_ref = Vec3{0.0, 0.0, 1.0};
            yI = y_ref - dot(y_ref, xI_cmd) * xI_cmd;
        }
        yI = yI / norm(yI);
        const Vec3 zI = cross(xI_cmd, yI);

        Mat<3,3> R_cmd_BI{};
        R_cmd_BI(0,0) = xI_cmd.x; R_cmd_BI(0, 1) = yI.x; R_cmd_BI(0,2) = zI.x;
        R_cmd_BI(1,0) = xI_cmd.y; R_cmd_BI(1,1) = yI.y; R_cmd_BI(1,2) = zI.y;
        R_cmd_BI(2,0) = xI_cmd.z; R_cmd_BI(2,1) = yI.z; R_cmd_BI(2,2) = zI.z;

        Quaternion q_cmd_BI = rotationToQuaternion(R_cmd_BI);
        q_cmd_BI.normalize();

        if (norm(Vec3{a_thrust_cmd_I.x, a_thrust_cmd_I.y, 0.0}) < 1e-4) {
            q_cmd_BI = nav.q_BI;   // no lateral thrust direction requested
        }

        if (dot(q_cmd_BI, nav.q_BI) < 0.0) {
            q_cmd_BI = q_cmd_BI * -1.0;
        }


        Quaternion q_err = nav.q_BI.conjugate() * q_cmd_BI;
        if (q_err.w < 0.0) {
            q_err = q_err * -1.0;
        }
        q_err.normalize();
        cmd.q_err = q_err;
        const Vec3 e{2.0 * q_err.x, 2.0 * q_err.y, 2.0 * q_err.z};

        const Vec3 tau_cmd_body = K_.kp_rot * e - K_.kd_rot * nav.omega_body;
        cmd.tau_cmd = tau_cmd_body;

        cmd.throttle_cmd = thrust_req / max_thrust_;

        if (thrust_req < min_thrust_for_control_ ||
            norm(Vec3{a_thrust_cmd_I.x, a_thrust_cmd_I.y, 0.0}) < lateral_accel_deadband_) {
            cmd.gimbal_pitch_cmd = 0.0;
            cmd.gimbal_yaw_cmd = 0.0;
        }
        else {
            const double torque_scale = std::max(std::abs(lever_arm_) * cmd.throttle_cmd * max_thrust_, 1e-6);

            cmd.raw_gimbal_pitch_cmd = -tau_cmd_body.y / torque_scale;
            double clamped_gimbal_pitch_cmd = std::clamp(
                cmd.raw_gimbal_pitch_cmd, -params_.gimbal_limit, params_.gimbal_limit);
            cmd.raw_filtered_gimbal_pitch_cmd =
                alpha_ * prev_cmd.raw_filtered_gimbal_pitch_cmd + (1.0 - alpha_) * clamped_gimbal_pitch_cmd;
            cmd.gimbal_pitch_cmd = limitRate(
                cmd.raw_filtered_gimbal_pitch_cmd,
                prev_cmd.gimbal_pitch_cmd,
                params_.gimbal_rate_limit,
                dt);

            cmd.raw_gimbal_yaw_cmd = -tau_cmd_body.z / torque_scale;
            double clamped_gimbal_yaw_cmd = std::clamp(
                cmd.raw_gimbal_yaw_cmd, -params_.gimbal_limit, params_.gimbal_limit);
            cmd.raw_filtered_gimbal_yaw_cmd =
                alpha_ * prev_cmd.raw_filtered_gimbal_yaw_cmd + (1.0 - alpha_) * clamped_gimbal_yaw_cmd;
            cmd.gimbal_yaw_cmd = limitRate(
                cmd.raw_filtered_gimbal_yaw_cmd,
                prev_cmd.gimbal_yaw_cmd,
                params_.gimbal_rate_limit,
                dt);
        }
        cmd.engine_enable_cmd = true;
        return cmd;
    }

private:
    LandingGains K_;
    double max_thrust_;
    double lever_arm_;
    const SimpleActuatorParams params_;
    double min_thrust_for_control_ = 0.1 * max_thrust_;
    Vec3 g0_I_ = {0.0, 0.0, -9.81};
    double max_tilt_rad_ = 10 * M_PI / 180.0;
    double lateral_accel_deadband_ = 1e-4;
    double alpha_ = 0.95;

    mutable bool initialized_ = false;
    mutable Vec3 xI_cmd{1.0, 0.0, 0.0};
};

#endif //MISSILE6DOF_PDLANDINGCONTROLLER_H
//
// Created by Ashton Lowenstein on 5/11/26.
//

#ifndef MISSILE6DOF_EKFNAVIGATION_H
#define MISSILE6DOF_EKFNAVIGATION_H
#include "GNCInterfaces/INavigationModel.h"
#include "ModelInterfaces/IGravityModel.h"
#include "MathTypes/MatrixXd.h"
#include "MathTypes/RotationConversions.h"

class EkfNavigation : public INavigationModel {
public:
    EkfNavigation(
        std::shared_ptr<IGravityModel> gravity,
        const Mat<9,9>& Q,
        const Mat<3,3>& R_gps,
        double max_thrust,
        double throttle_time_const,
        double dry_mass
    ) : gravity_(std::move(gravity)), Q_(Q), R_gps_(R_gps), max_thrust_(max_thrust),
        throttle_time_const_(throttle_time_const), dry_mass_(dry_mass) {
        if (!gravity_) {
            throw std::invalid_argument("EkfNavigation requires a gravity model");
        }
    }

    [[nodiscard]] NavigationState initializeFromTruth(const State& truth) const override {
        NavigationState output{};
        output.pos_inertial = truth.pos_inertial;
        output.vel_inertial = truth.vel_inertial;
        output.q_BI = truth.q_BI;
        output.omega_body = truth.omega_body;
        output.mass_est = truth.mass;

        for (std::size_t i = 0; i < 3; ++i) {
            output.P(i,i) = 1.0;
            output.P(i+3,i+3) = 1.0;
            output.P(i+6,i+6) = 0.1;
        }

        return output;
    }

    [[nodiscard]] NavigationState predict(
        double dt,
        const NavigationState& prev, // previous EKF estimate x^hat_{k-1 | k-1}
        const ImuMeasurement& imu) const {

        NavigationState prediction = prev;

        // 1. Bias-correct IMU
        const Vec3 omega_body = imu.gyro_body;// - prev.gyro_bias_body;

        const Vec3 specific_force_body =
            imu.accel_body;// - prev.accel_bias_body;

        // 2. Propagate attitude, same as ImuNavigation
        const Quaternion omega_q = toQuaternion(omega_body);
        const Quaternion q_dot = prev.q_BI * omega_q * 0.5;
        const Quaternion q_new = prev.q_BI + q_dot * dt;
        prediction.q_BI = q_new / q_new.norm();

        prediction.omega_body = omega_body;

        // 3. Rotate specific force into inertial frame
        const Vec3 specific_force_inertial =
            prediction.q_BI.rotateVector(specific_force_body);

        // 4. Add gravity evaluated at estimated position
        const Vec3 gravity_inertial =
            gravity_->compute(prev.pos_inertial).accel_inertial;

        const Vec3 accel_inertial =
            specific_force_inertial + gravity_inertial;

        // 5. Propagate translational state
        prediction.vel_inertial =
            prev.vel_inertial + dt * accel_inertial;

        prediction.pos_inertial =
            prev.pos_inertial
            + dt * prev.vel_inertial
            + 0.5 * dt * dt * accel_inertial;

        // 6. Propagate estimate covariance
        Mat<3,3> R_q = toRotationMatrix(prediction.q_BI);

        Mat<9,9> F = identity<9>();
        F(0,3) = dt; F(1,4) = dt; F(2,5) = dt;
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                F(3+i, 6+j) = -dt * R_q(i,j);
            }
        }

        prediction.P = F * prev.P * transpose(F) + Q_;
        prediction.P = 0.5 * (prediction.P + transpose(prediction.P)); //enforce symmetry

        return prediction;
    }

    [[nodiscard]] NavigationState correctGPS(const NavigationState& predicted, const GpsMeasurement& gps) const {
        // Continues the EKF process
        NavigationState corrected = predicted;

        // 7. Measurement and innovation
        Mat<3,9> H = zeros<3,9>();
        H(0,0) = 1.0; H(1,1) = 1.0; H(2,2) = 1.0;

        const Vec3 innovation = gps.pos_inertial - predicted.pos_inertial;
        const Mat<3,1> y = toVecXd(innovation);

        const Mat<3,3> S = H * predicted.P * transpose(H) + R_gps_;

        // 8. Kalman gain: K = P * H^T * S^-1
        // Accomplish by solving linear equation for K^T instead of inverting S
        const Mat<9,3> K = transpose(solve(transpose(S), H * transpose(predicted.P)));

        // 9. Correct predicted state
        Mat<9,1> dx = K * y;
        corrected.pos_inertial += Vec3{dx(0,0), dx(1,0), dx(2,0)};
        corrected.vel_inertial += Vec3{dx(3,0), dx(4,0), dx(5,0)};
        corrected.accel_bias_body += Vec3{dx(6,0), dx(7,0), dx(8,0)};

        // 10. Correct state estimate covariance
        corrected.P = (identity<9>() - K * H) * predicted.P * transpose(identity<9>() - K * H) +
            K * R_gps_ * transpose(K);
        corrected.P = 0.5 * (corrected.P + transpose(corrected.P)); //enforce symmetry

        return corrected;
    }

    [[nodiscard]] NavigationState estimate(
        double t,
        double dt,
        const State& state,
        const NavigationState& prev,
        const ImuMeasurement& sensors,
        const ControlCommand& last_cmd,
        const std::optional<GpsMeasurement>& gps
    ) const override {
        (void)t; (void)state;

        NavigationState x_hat = predict(dt, prev, sensors);

        if (gps.has_value()) {
            x_hat = correctGPS(x_hat, gps.value());
        }

        double throttle_cmd_sat = std::clamp(last_cmd.throttle_cmd, 0.0, 1.0);

        double throttle_dot =
            (throttle_cmd_sat - prev.throttle_est) / throttle_time_const_;

        double throttle_est_next =
            prev.throttle_est + throttle_dot * dt;

        double thrust_est =
            throttle_est_next * max_thrust_;

        double mdot_est = thrust_est * throttle_cmd_sat;

        x_hat.mass_est = std::max(dry_mass_, prev.mass_est - mdot_est * dt);
        x_hat.throttle_est = throttle_est_next;

        return x_hat;

    }

private:
    std::shared_ptr<IGravityModel> gravity_;
    Mat<9,9> Q_;
    Mat<3,3> R_gps_;
    double max_thrust_;
    double throttle_time_const_;
    double dry_mass_;
};

#endif //MISSILE6DOF_EKFNAVIGATION_H
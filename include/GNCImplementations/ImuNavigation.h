//
// Created by Ashton Lowenstein on 5/6/26.
//

#ifndef MISSILE6DOF_IMUNAVIGATION_H
#define MISSILE6DOF_IMUNAVIGATION_H

#include <memory>

#include "../GNCInterfaces/INavigationModel.h"
#include "ModelInterfaces/IGravityModel.h"

class ImuNavigation : public INavigationModel {
public:
    explicit ImuNavigation(std::shared_ptr<const IGravityModel> gravity)
        : gravity_(std::move(gravity)) {
        if (!gravity_) {
            throw std::invalid_argument("ImuNavigation requires a gravity model");
        }
    }

    [[nodiscard]] NavigationState initializeFromTruth(const State& truth) const override {
        return {truth.pos_inertial, truth.vel_inertial, truth.q_BI, truth.omega_body};
    }

    [[nodiscard]] NavigationState estimate(
            double t,
            double dt,
            const State& state,
            const NavigationState& prev,
            const ImuMeasurement& sensors,
            const std::optional<GpsMeasurement>& gps
        ) const override {

        (void)t; (void)state; (void)gps;

        NavigationState update = prev;

        update.omega_body = sensors.gyro_body;
        const Quaternion omega_q = toQuaternion(update.omega_body);
        const Quaternion q_dot = prev.q_BI * omega_q * 0.5;
        const Quaternion q_new = prev.q_BI + q_dot * dt;
        update.q_BI = q_new / q_new.norm();

        Vec3 accel_inertial = update.q_BI.rotateVector(sensors.accel_body);
        accel_inertial += gravity_->compute(prev.pos_inertial).accel_inertial;

        update.vel_inertial = prev.vel_inertial + dt * accel_inertial;
        update.pos_inertial = prev.pos_inertial + dt * prev.vel_inertial + 0.5 * dt * dt * accel_inertial;

        return update;
    }
private:
    std::shared_ptr<const IGravityModel> gravity_;
};

#endif //MISSILE6DOF_IMUNAVIGATION_H
//
// Created by Ashton Lowenstein on 4/15/26.
//

#ifndef MISSILE6DOF_SIMPLEGIMBALEDPROPULSIONMODEL_H
#define MISSILE6DOF_SIMPLEGIMBALEDPROPULSIONMODEL_H

#include "../ModelInterfaces/IPropulsionModel.h"
#include <cmath>

class SimpleGimbaledPropulsionModel : public IPropulsionModel {
public:
    SimpleGimbaledPropulsionModel(
        double max_thrust,
        double isp,
        const Vec3& engine_location_body,
        double dry_mass)
            : max_thrust_(max_thrust),
              isp_(isp),
              engine_location_body_(engine_location_body),
              dry_mass_(dry_mass) {}

    [[nodiscard]] PropulsionLoads compute(
        const State& state,
        const FlightCondition& fc,
        const ActuatorOutput& act,
        const MassProperties& mp) const override
    {
        PropulsionLoads loads{};
        // std::cout << "engine enable: " <<act.engine_enable << std::endl;
        // std::cout << "throttle: " << act.throttle << std::endl;
        // std::cout << "mass: " << mp.mass << std::endl;
        // std::cout << "max thrust: " << max_thrust_ << std::endl;
        // std::cout << "dry mass: " << dry_mass_ << std::endl;

        if (!act.engine_enable || act.throttle <= 0.0 || mp.mass <= dry_mass_) {
            return loads;
        }
        // std::cout << "expr = " << (act.throttle * max_thrust_) << "\n";
        const double thrust = act.throttle * max_thrust_;
        loads.thrust = thrust;
        const Vec3 thrust_dir_body = gimbalDirectionBody(act.gimbal_pitch, act.gimbal_yaw);

        loads.force_body = thrust * thrust_dir_body;

        const Vec3 r_com_to_engine = engine_location_body_ - mp.com_body;
        loads.moment_body = cross(r_com_to_engine, loads.force_body);

        loads.mdot = thrust / (isp_ * 9.80665);
        loads.engine_active = true;

        return loads;
    }
private:
    double max_thrust_;
    double isp_;
    Vec3 engine_location_body_;
    double dry_mass_;

    static Vec3 gimbalDirectionBody(double gimbal_pitch, double gimbal_yaw)
    {
        const double cp = std::cos(gimbal_pitch);
        const double sp = std::sin(gimbal_pitch);
        const double cy = std::cos(gimbal_yaw);
        const double sy = std::sin(gimbal_yaw);

        return Vec3{
            cy * cp,
            sy * cp,
            -sp
        };
    }
};

#endif //MISSILE6DOF_SIMPLEGIMBALEDPROPULSIONMODEL_H
//
// Created by Ashton Lowenstein on 4/17/26.
//

#ifndef MISSILE6DOF_SIMPLEACTUATORMODEL_H
#define MISSILE6DOF_SIMPLEACTUATORMODEL_H

#include <algorithm>

#include "../ModelInterfaces/IActuatorModel.h"

class SimpleActuatorModel : public IActuatorModel {
public:
    explicit SimpleActuatorModel(const SimpleActuatorParams& params) : params_(params) {}

    [[nodiscard]] ActuatorOutput output(
        const State& state,
        const ControlCommand& cmd
    ) const override {
        ActuatorOutput output;

        output.throttle = std::clamp(state.actuators.throttle, 0.0, 1.0);

        output.gimbal_pitch = std::clamp(
            state.actuators.gimbal_pitch,
            -params_.gimbal_limit,
            params_.gimbal_limit
        );

        output.gimbal_yaw = std::clamp(
            state.actuators.gimbal_yaw,
            -params_.gimbal_limit,
            params_.gimbal_limit);

        output.engine_enable = cmd.engine_enable_cmd;

        return output;
    }

    [[nodiscard]] PropulsionActuatorDerivative derivatives(
        const State& state,
        const ControlCommand& cmd) const override {
        PropulsionActuatorDerivative deriv;

        // Throttle

        const double throttle_cmd = std::clamp(cmd.throttle_cmd, 0.0, 1.0);
        const double throttle = std::clamp(state.actuators.throttle, 0.0, 1.0);

        deriv.throttle_dot = (throttle_cmd - throttle) / params_.throttle_time_constant;
        deriv.throttle_dot = std::clamp(
            deriv.throttle_dot,
            -params_.throttle_rate_limit,
            params_.throttle_rate_limit);


        // Pitch

        const double gimbal_pitch_cmd = std::clamp(
            cmd.gimbal_pitch_cmd,
            -params_.gimbal_limit,
            params_.gimbal_limit);
        const double gimbal_pitch = std::clamp(
            state.actuators.gimbal_pitch,
            -params_.gimbal_limit,
            params_.gimbal_limit);

        deriv.gimbal_pitch_dot = (gimbal_pitch_cmd - gimbal_pitch) / params_.gimbal_time_constant;
        deriv.gimbal_pitch_dot = std::clamp(
            deriv.gimbal_pitch_dot,
            -params_.gimbal_rate_limit,
            params_.gimbal_rate_limit);

        // if the state is already at the gimbal limit and the command is beyond the limit, the first order derivative
        // update could still try to push outward
        if (gimbal_pitch >= params_.gimbal_limit && deriv.gimbal_pitch_dot > 0.0) {
            deriv.gimbal_pitch_dot = 0.0;
        }
        if (gimbal_pitch <= -params_.gimbal_limit && deriv.gimbal_pitch_dot < 0.0) {
            deriv.gimbal_pitch_dot = 0.0;
        }

        // Yaw

        const double gimbal_yaw_cmd = std::clamp(
            cmd.gimbal_yaw_cmd,
            -params_.gimbal_limit,
            params_.gimbal_limit);
        const double gimbal_yaw = std::clamp(
            state.actuators.gimbal_yaw,
            -params_.gimbal_limit,
            params_.gimbal_limit);

        deriv.gimbal_yaw_dot = (gimbal_yaw_cmd - gimbal_yaw) / params_.gimbal_time_constant;
        deriv.gimbal_yaw_dot = std::clamp(
            deriv.gimbal_yaw_dot,
            -params_.gimbal_rate_limit,
            params_.gimbal_rate_limit);

        if (gimbal_yaw >= params_.gimbal_limit && deriv.gimbal_yaw_dot > 0.0) {
            deriv.gimbal_yaw_dot = 0.0;
        }
        if (gimbal_yaw <= -params_.gimbal_limit && deriv.gimbal_yaw_dot < 0.0) {
            deriv.gimbal_yaw_dot = 0.0;
        }

        return deriv;
    }
private:
    SimpleActuatorParams params_;
};

#endif //MISSILE6DOF_SIMPLEACTUATORMODEL_H
//
// Created by Ashton Lowenstein on 4/24/26.
//

#include "../include/ModelImplementations/SimpleActuatorModel.h"
#include "../include/core/State.h"
#include "test_utils.h"

int main() {
    SimpleActuatorParams params{};
    params.throttle_time_constant = 0.2;
    params.throttle_rate_limit = 10.0;
    params.gimbal_time_constant = 0.1;
    params.gimbal_rate_limit = 2.0;
    params.gimbal_limit = 10.0 * M_PI / 180.0;

    SimpleActuatorModel actuator(params);

    State s{};
    s.actuators.throttle = 0.0;
    s.actuators.gimbal_pitch = 0.0;
    s.actuators.gimbal_yaw = 0.0;

    ControlCommand cmd{};
    cmd.throttle_cmd = 1.0;
    cmd.gimbal_pitch_cmd = 5.0 * M_PI / 180.0;
    cmd.gimbal_yaw_cmd = 0.0;
    cmd.engine_enable_cmd = true;

    auto out = actuator.output(s, cmd);
    auto d = actuator.derivatives(s, cmd);

    expectNear(out.throttle, 0.0, 1e-12, "initial throttle output");
    expectTrue(d.throttle_dot > 0.0, "throttle derivative positive");
    expectTrue(d.gimbal_pitch_dot > 0.0, "pitch derivative positive");
    expectNear(d.gimbal_yaw_dot, 0.0, 1e-12, "yaw derivative zero");

    std::cout << "test_actuator passed\n";
    return 0;
}
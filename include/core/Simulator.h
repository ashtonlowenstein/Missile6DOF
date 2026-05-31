//
// Created by Ashton Lowenstein on 3/25/26.
//

#ifndef MISSILE6DOF_SIMULATOR_H
#define MISSILE6DOF_SIMULATOR_H

#include "../dynamics/MissileDynamics.h"
#include <string>

#include "../logging/LogRecord.h"
#include "../GNCInterfaces/IControllerModel.h"
#include "../GNCInterfaces/IGuidanceModel.h"
#include "../GNCInterfaces/INavigationModel.h"
#include "../sensors/IImuModel.h"
#include "sensors/IGpsModel.h"

class Simulator {
public:
    Simulator(
        const MissileDynamics& dynamics,
        std::unique_ptr<IImuModel> imu,
        std::unique_ptr<IGpsModel> gps,
        std::unique_ptr<IGuidanceModel> guidance,
        std::unique_ptr<INavigationModel> navigation,
        std::unique_ptr<IControllerModel> controller,
        double dt,
        double t_end);

    [[nodiscard]] State run(const State& initial_state) const;

    [[nodiscard]] State run(
        const State &initial_state,
        const std::string &output_path) const;

    [[nodiscard]] State run_gain_tuning(
        const State &initial_state,
        const std::string &output_path) const;

private:
    const MissileDynamics& dynamics_;
    std::unique_ptr<IImuModel> imu_;
    std::unique_ptr<IGpsModel> gps_;
    std::unique_ptr<IGuidanceModel> guidance_;
    std::unique_ptr<INavigationModel> navigation_;
    std::unique_ptr<IControllerModel> controller_;
    double dt_;
    double t_end_;

    [[nodiscard]] LogRecord fillLogRecord(
        double t,
        const State& state,
        const Derivative& ds,
        const ControlCommand& cmd,
        const DynamicsContext& ctx,
        const NavigationState& nav,
        const ImuMeasurement& measurement) const;
};


#endif //MISSILE6DOF_SIMULATOR_H
//
// Created by Ashton Lowenstein on 4/14/26.
//

#ifndef MISSILE6DOF_LOADSAGGREGATOR_H
#define MISSILE6DOF_LOADSAGGREGATOR_H

#include "../ModelInterfaces/IAeroModel.h"
#include "../ModelInterfaces/IGravityModel.h"
#include "../ModelInterfaces/IPropulsionModel.h"
#include "../core/State.h"

class LoadsAggregator {
public:
    LoadsAggregator(
        const IGravityModel& gravity_model,
        //const ActuatorModel& actuator_model,
        const IPropulsionModel& propulsion_model,
        const IAeroModel& aero_model
    );

    [[nodiscard]] TotalLoads compute(
        double t,
        const State& state,
        const FlightCondition& fc,
        const MassProperties& mp,
        const ActuatorOutput& act
    ) const;

private:
    const IGravityModel& gravity_model_;
    //const ActuatorModel& actuator_model_;
    const IPropulsionModel& propulsion_model_;
    const IAeroModel& aero_model_;
};


#endif //MISSILE6DOF_LOADSAGGREGATOR_H
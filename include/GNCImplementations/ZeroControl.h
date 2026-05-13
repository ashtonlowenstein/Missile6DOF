//
// Created by Ashton Lowenstein on 5/6/26.
//

#ifndef MISSILE6DOF_ZEROCONTROL_H
#define MISSILE6DOF_ZEROCONTROL_H

#include "../GNCInterfaces/IControllerModel.h"

class ZeroControl : public IControllerModel {
public:
    [[nodiscard]] ControlCommand compute(
        double t,
        const NavigationState& nav,
        const GuidanceCommand& guid) const override {

        (void)t; (void)nav;

        return {
            guid.throttle_cmd,
            0.0,
            0.0,
            true};
    }
};

#endif //MISSILE6DOF_ZEROCONTROL_H
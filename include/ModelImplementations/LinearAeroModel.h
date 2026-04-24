//
// Created by Ashton Lowenstein on 3/25/26.
//

#ifndef MISSILE6DOF_LINEARAEROMODEL_H
#define MISSILE6DOF_LINEARAEROMODEL_H

#include "../ModelInterfaces/IAeroModel.h"

struct LinearAeroParams {
    double reference_area{1.0};     // S
    double reference_length{1.0};   // d

    double CD0{0.1};
    double CD_alpha2{0.0};
    double CD_beta2{0.0};

    double CY_beta{0.0};
    double CY_r{0.0};

    double CZ_alpha{0.0};
    double CZ_q{0.0};

    double Cm_alpha{0.0};
    double Cm_q{0.0};

    double Cn_beta{0.0};
    double Cn_r{0.0};
};

class LinearAeroModel : public IAeroModel {
public:
    explicit LinearAeroModel(const LinearAeroParams& params) : params_(params) {}

    [[nodiscard]] AeroLoads computeLoads(
        double t,
        const State &state,
        const FlightCondition &fc,
        const ActuatorOutput& act,
        const MassProperties& mp
    ) const override
    {
        (void) t; (void) state;

        AeroLoads loads{};

        const double V = fc.air_speed;
        const double qbar = fc.qbar;

        if (V < 1e-8 || qbar <= 0.0) {
            return loads;
        }

        const double d = params_.reference_length;
        const double S = params_.reference_area;

        const double p = fc.omega_body.x;
        const double q = fc.omega_body.y;
        const double r = fc.omega_body.z;

        (void) p; // not including roll model yet

        const double q_hat = q * d / (2.0 * V);
        const double r_hat = r * d / (2.0 * V);

        const double CX =
            -(params_.CD0 + params_.CD_alpha2 * fc.alpha * fc.alpha + params_.CD_beta2 * fc.beta * fc.beta);
        const double CY = params_.CY_beta * fc.beta + params_.CY_r * r_hat;
        const double CZ = params_.CZ_alpha * fc.alpha + params_.CZ_q * q_hat;
        const double Cm = params_.Cm_alpha * fc.alpha + params_.Cm_q * q_hat;
        const double Cn = params_.Cn_beta * fc.beta + params_.Cn_r * r_hat;

        loads.force_body = {
            qbar * S * CX,
            qbar * S * CY,
            qbar * S * CZ,
        };

        loads.moment_body = {
            0.0,
            qbar * S * d * Cm,
            qbar * S * d * Cn,
        };

        return loads;
    }
private:
    LinearAeroParams params_;
};

#endif //MISSILE6DOF_LINEARAEROMODEL_H
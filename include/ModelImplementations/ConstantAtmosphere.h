//
// Created by Ashton Lowenstein on 3/25/26.
//

#ifndef MISSILE6DOF_CONSTANTATMOSPHERE_H
#define MISSILE6DOF_CONSTANTATMOSPHERE_H
#include "../ModelInterfaces/IAtmosphere.h"

class ConstantAtmosphereModel : public IAtmosphereModel {
    // Units:
    // density: kg/m^3
    // pressure: Pa
    // temperature: K
    // speed_of_sound: m/s
    // wind_inertial: m/s
public:
    ConstantAtmosphereModel(double density,
                            double pressure,
                            double temperature,
                            double speed_of_sound,
                            Vec3 wind_inertial)
        : density_(density),
          pressure_(pressure),
          temperature_(temperature),
          speed_of_sound_(speed_of_sound),
          wind_inertial_(wind_inertial) {}
    [[nodiscard]] AtmosphereState sample(const Vec3& pos_inertial) const override {
        (void) pos_inertial;
        return AtmosphereState{
            density_,
            pressure_,
            temperature_,
            speed_of_sound_,
            wind_inertial_
        };
    }

private:
    const double density_;
    const double pressure_;
    const double temperature_;
    const double speed_of_sound_;
    const Vec3 wind_inertial_;
};

#endif //MISSILE6DOF_CONSTANTATMOSPHERE_H
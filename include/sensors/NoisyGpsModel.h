//
// Created by Ashton Lowenstein on 5/12/26.
//

#ifndef MISSILE6DOF_NOISYBIASEDGPSMODEL_H
#define MISSILE6DOF_NOISYBIASEDGPSMODEL_H

#include "IGpsModel.h"
#include <random>

class NoisyGpsModel : public IGpsModel {
public:
    explicit NoisyGpsModel(double sigma_pos)
        : rng_(std::random_device{}()),
          normal_(0, sigma_pos) {}

    // explicit NoisyGpsModel(double sigma, uint32_t seed = 0)
    //     : rng_(seed),
    //       normal_(0.0, sigma) {}

    [[nodiscard]] std::optional<GpsMeasurement> measure(
        double t,
        const State& truth) const override {

        const Vec3 noise{normal_(rng_), normal_(rng_), normal_(rng_)};

        return GpsMeasurement{
            truth.pos_inertial + noise,
            true
        };
    }

private:
    mutable std::mt19937 rng_;
    mutable std::normal_distribution<double> normal_;
};

#endif //MISSILE6DOF_NOISYBIASEDGPSMODEL_H
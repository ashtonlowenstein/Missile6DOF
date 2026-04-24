//
// Created by Ashton Lowenstein on 4/24/26.
//

#ifndef MISSILE6DOF_TEST_UTILS_H
#define MISSILE6DOF_TEST_UTILS_H

#include <cmath>
#include <iostream>
#include <string>
#include "../include/MathTypes/Mat3.h"

inline void expectNear(double a, double b, double tol, const std::string& msg) {
    if (std::abs(a - b) > tol) {
        std::cerr << "FAIL: " << msg
                  << " expected " << b
                  << " got " << a << "\n";
        std::exit(1);
    }
}

inline void expectTrue(bool cond, const std::string& msg) {
    if (!cond) {
        std::cerr << "FAIL: " << msg << "\n";
        std::exit(1);
    }
}

inline void expectVecNear(const Vec3& a, const Vec3& b, double tol, const std::string& msg) {
    expectNear(a.x, b.x, tol, msg + " x");
    expectNear(a.y, b.y, tol, msg + " y");
    expectNear(a.z, b.z, tol, msg + " z");
}

#endif
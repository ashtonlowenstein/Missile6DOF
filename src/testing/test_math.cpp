//
// Created by Ashton Lowenstein on 4/24/26.
//

#include "test_utils.h"

#include "../include/MathTypes/Vec3.h"
#include "../include/MathTypes/Mat3.h"
#include "../include/MathTypes/Quaternion.h"

int main() {
    constexpr double tol = 1e-10;

    const Vec3 ex{1,0,0};
    const Vec3 ey{0,1,0};
    const Vec3 ez{0,0,1};

    expectVecNear(cross(ex, ey), ez, tol, "cross ex ey");
    expectVecNear(cross(ey, ex), Vec3{0,0,-1}, tol, "cross ey ex");
    expectNear(dot(ex, ey), 0.0, tol, "dot orthogonal");

    const Mat3 I = Mat3::diagonal(1.0, 1.0, 1.0);
    const Vec3 v{1.2, -3.4, 5.6};
    expectVecNear(I * v, v, tol, "identity matvec");

    const Mat3 A{
        1, 2, 3,
        0, 1, 4,
        5, 6, 0
    };
    const Mat3 Ainv = inverse(A);
    Mat3 should_be_I = A * Ainv;

    expectNear(should_be_I(0,0), 1.0, 1e-9, "inverse 00");
    expectNear(should_be_I(1,1), 1.0, 1e-9, "inverse 11");
    expectNear(should_be_I(2,2), 1.0, 1e-9, "inverse 22");

    const Quaternion qz = Quaternion(Vec3{0,0,1}, M_PI / 2.0);
    expectVecNear(qz.rotateVector(ex), ey, 1e-9, "quat rotate z 90");

    std::cout << "test_math passed\n";
    return 0;
}
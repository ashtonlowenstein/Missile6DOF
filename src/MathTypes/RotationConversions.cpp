//
// Created by Ashton Lowenstein on 5/18/26.
//

#include "../include/MathTypes/RotationConversions.h"

Mat<3,3> toRotationMatrix(Quaternion& q){
    Mat<3,3> R{};
    q.normalize();

    R(0,0) = 1.0 - 2.0 * (q.y * q.y + q.z * q.z); R(0,1) = 2.0 * (q.x * q.y - q.z * q.w); R(0,2) = 2.0 * (q.x * q.z + q.y * q.w);
    R(1,0) = 2.0 * (q.x * q.y + q.z * q.w); R(1,1) = 1.0 - 2.0 * (q.x * q.x + q.z * q.z); R(1,2) = 2.0 * (q.y * q.z - q.x * q.w);
    R(2,0) = 2.0 * (q.x * q.z - q.y * q.w); R(2,1) = 2.0 * (q.y * q.z + q.x * q.w); R(2,2) = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);

    return R;
}

Quaternion rotationToQuaternion(const Mat<3,3>& A) {
    constexpr double eps = 1e-12;

    double cos_theta = 0.5 * (trace(A) - 1.0);
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);

    const double theta = std::acos(cos_theta);

    if (std::abs(theta) < eps) {
        return {1.0, 0.0, 0.0, 0.0};
    }

    const double s = std::sin(theta);

    Vec3 axis{
        (A(2,1) - A(1,2)) / (2.0 * s),
        (A(0,2) - A(2,0)) / (2.0 * s),
        (A(1,0) - A(0,1)) / (2.0 * s)
    };

    axis = axis / norm(axis);

    return {axis, theta};
}

Vec3 toVec3(const Mat<3,1>& A) {
    return {
        A(0,0),
        A(1,0),
        A(2,0)
    };
}

Vec3 toVec3(const Mat<1,3>& A) {
    return {
        A(0,0),
        A(0,1),
        A(0,2)
    };
}

Mat<3,1> toVecXd(const Vec3& v) {
    Mat<3,1> output{};
    output(0,0) = v.x; output(1,0) = v.y; output(2,0) = v.z;
    return output;
}
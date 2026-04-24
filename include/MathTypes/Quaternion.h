//
// Created by Ashton Lowenstein on 3/24/26.
//

#ifndef MISSILE6DOF_QUATERNION_H
#define MISSILE6DOF_QUATERNION_H
#include "Vec3.h"


class Quaternion {
public:
    double w;
    double x;
    double y;
    double z;

    Quaternion();
    Quaternion(double w, double x, double y, double z);
    Quaternion(const Vec3& axis, double angle);

    Quaternion operator+(const Quaternion& q) const;
    Quaternion operator-(const Quaternion& q) const;
    Quaternion operator*(const Quaternion& q) const;
    Quaternion operator*(double lambda) const;
    Quaternion operator/(double lambda) const;

    Quaternion& operator+=(const Quaternion& q);
    Quaternion& operator-=(const Quaternion& q);

    [[nodiscard]] double norm() const;
    void normalize();
    [[nodiscard]] Quaternion conjugate() const;
    [[nodiscard]] Vec3 rotateVector(const Vec3& v) const;
};

Vec3 toVector(const Quaternion& q);
Quaternion toQuaternion(const Vec3& v);

#endif //MISSILE6DOF_QUATERNION_H
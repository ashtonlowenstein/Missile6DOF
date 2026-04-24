//
// Created by Ashton Lowenstein on 3/24/26.
//

#include "../../include/MathTypes/Quaternion.h"

#include <valarray>
#include <cmath>

Quaternion::Quaternion() : w(0.0), x(0.0), y(0.0), z(0.0) {}

Quaternion::Quaternion(const double w, const double x, const double y, const double z)
    : w(w), x(x), y(y), z(z) {}

Quaternion::Quaternion(const Vec3 &axis, double angle)
    : w(cos(angle / 2.0)),
      x(axis.x * sin(angle / 2.0)),
      y(axis.y * sin(angle / 2.0)),
      z(axis.z * sin(angle / 2.0))
{}


Quaternion Quaternion::operator+(const Quaternion &q) const {
    return {w + q.w, x + q.x, y + q.y, z + q.z};
}

Quaternion Quaternion::operator-(const Quaternion &q) const {
    return {w - q.w, x - q.x, y - q.y, z - q.z};
}

Quaternion Quaternion::operator*(const Quaternion &q) const {
    return {
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w,
    };
}

Quaternion Quaternion::operator*(const double lambda) const {
    return {w * lambda, x * lambda, y * lambda, z * lambda};
}

Quaternion Quaternion::operator/(const double lambda) const {
    return {w / lambda, x / lambda, y / lambda, z / lambda, };
}

Quaternion& Quaternion::operator+=(const Quaternion &q) {
    w += q.w;
    x += q.x;
    y += q.y;
    z += q.z;
    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion &q) {
    w -= q.w;
    x -= q.x;
    y -= q.y;
    z -= q.z;
    return *this;
}

double Quaternion::norm() const {
    return std::sqrt(w * w + x * x + y * y + z * z);
}

void Quaternion::normalize() {
    if (const double n = norm(); n > 0.0) {
        w /= n;
        x /= n;
        y /= n;
        z /= n;
    }
}

Quaternion Quaternion::conjugate() const {
    return Quaternion{w, -x, -y, -z};
}

Vec3 toVector(const Quaternion& q) {
    return Vec3{q.x, q.y, q.z};
}

Quaternion toQuaternion(const Vec3& v) {
    return Quaternion{0.0, v.x, v.y, v.z};
}

Vec3 Quaternion::rotateVector(const Vec3& v) const {
    if (const double n = norm(); n > 0.0) {
        const Quaternion v_q = toQuaternion(v);
        const Quaternion q_rot = *this / n;
        return toVector(q_rot * v_q * q_rot.conjugate());
    }
    return v;
}
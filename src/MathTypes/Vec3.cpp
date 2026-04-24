//
// Created by Ashton Lowenstein on 3/24/26.
//

#include "../../include/MathTypes/Vec3.h"
#include <valarray>

Vec3::Vec3() : x(0.0), y(0.0), z(0.0) {}

Vec3::Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

Vec3 Vec3::operator+(const Vec3 &other) const {
    return {x + other.x, y + other.y, z + other.z};
}

Vec3 Vec3::operator-(const Vec3 &other) const {
    return {x - other.x, y - other.y, z - other.z};
}

Vec3 Vec3::operator*(double lambda) const {
    return {x * lambda, y * lambda, z * lambda};
}

Vec3 Vec3::operator/(double lambda) const {
    return {x / lambda, y / lambda, z / lambda};
}

Vec3& Vec3::operator+=(const Vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vec3& Vec3::operator-=(const Vec3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vec3 operator*(double lambda, const Vec3& v) {
    return v * lambda;
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double norm(const Vec3& v) {
    return std::sqrt(dot(v, v));
}
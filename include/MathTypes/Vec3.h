//
// Created by Ashton Lowenstein on 3/24/26.
//

#ifndef MISSILE6DOF_VEC3_H
#define MISSILE6DOF_VEC3_H


class Vec3 {
public:
    double x;
    double y;
    double z;

    Vec3();
    Vec3(double x, double y, double z);

    Vec3 operator+(const Vec3 &other) const;
    Vec3 operator-(const Vec3 &other) const;
    Vec3 operator*(double lambda) const;
    Vec3 operator/(double lambda) const;

    Vec3& operator+=(const Vec3& other);
    Vec3& operator-=(const Vec3& other);
};

Vec3 operator*(double s, const Vec3& v);

double dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
double norm(const Vec3& v);


#endif //MISSILE6DOF_VEC3_H
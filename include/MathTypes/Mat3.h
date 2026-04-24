//
// Created by Ashton Lowenstein on 3/24/26.
//

#ifndef MISSILE6DOF_MAT3_H
#define MISSILE6DOF_MAT3_H

#include "Vec3.h"


class Mat3 {
public:
    double m[3][3];

    Mat3();
    Mat3(
        double m00, double m01, double m02,
        double m10, double m11, double m12,
        double m20, double m21, double m22
        );

    static Mat3 diagonal(double x, double y, double z);

    Vec3 operator*(const Vec3 &v) const;
    Mat3 operator*(const Mat3 &A) const;
    Mat3 operator+(const Mat3 &other) const;
    Mat3 operator-(const Mat3& other) const;
    Mat3 operator*(double lambda) const;

    double& operator()(int row, int col);
    const double& operator()(int row, int col) const;


};

Mat3 inverse(const Mat3 &A);

#endif //MISSILE6DOF_MAT3_H
//
// Created by Ashton Lowenstein on 3/24/26.
//

#include "../../include/MathTypes/Mat3.h"
#include <cmath>
#include <stdexcept>

Mat3::Mat3() : m{{0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0}} {}

Mat3::Mat3(double m00, double m01, double m02,
        double m10, double m11, double m12,
        double m20, double m21, double m22)
            : m{{m00, m01, m02},
                {m10, m11, m12},
                {m20, m21, m22}}
{}

Mat3 Mat3::diagonal(double x, double y, double z) {
    Mat3 M;
    M.m[0][0] = x; M.m[1][1] = y; M.m[2][2] = z;
    return M;
}

Vec3 Mat3::operator*(const Vec3 &v) const {
    return {
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
    };
}

Mat3 Mat3::operator*(const Mat3 &A) const {
    Mat3 out = Mat3();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                out.m[i][j] += m[i][k] * A.m[k][j];
            }
        }
    }
    return out;
}


Mat3 Mat3::operator+(const Mat3 &other) const {
    return {m[0][0] + other(0,0), m[0][1] + other(1,0), m[0][2] + other(2,0),
                m[1][0] + other(1,0), m[1][1] + other(1,1), m[1][2] + other(1,2),
                m[2][0] + other(2,0), m[2][1] + other(2,1), m[2][2] + other(2,2)};
}

Mat3 Mat3::operator-(const Mat3& other) const {
    return {m[0][0] - other(0,0), m[0][1] - other(1,0), m[0][2] - other(2,0),
                m[1][0] - other(1,0), m[1][1] - other(1,1), m[1][2] - other(1,2),
                m[2][0] - other(2,0), m[2][1] - other(2,1), m[2][2] - other(2,2)};
}

Mat3 Mat3::operator*(double lambda) const {
    return {m[0][0] * lambda, m[0][1] * lambda, m[0][2] * lambda,
            m[1][0] * lambda, m[1][1] * lambda, m[1][2] * lambda,
            m[2][0] * lambda, m[2][1] * lambda, m[2][2] * lambda};
}

double& Mat3::operator()(int row, int col) {
    return m[row][col];
}

const double& Mat3::operator()(int row, int col) const {
    return m[row][col];
}

Mat3 inverse(const Mat3& A)
{
    const double a = A(0,0), b = A(0,1), c = A(0,2);
    const double d = A(1,0), e = A(1,1), f = A(1,2);
    const double g = A(2,0), h = A(2,1), i = A(2,2);

    const double det =
        a*(e*i - f*h)
      - b*(d*i - f*g)
      + c*(d*h - e*g);

    // Guard against singular matrices
    if (std::abs(det) < 1e-12) {
        throw std::runtime_error("Matrix is singular or nearly singular");
    }

    const double inv_det = 1.0 / det;

    Mat3 inv;

    inv(0,0) =  (e*i - f*h) * inv_det;
    inv(0,1) = -(b*i - c*h) * inv_det;
    inv(0,2) =  (b*f - c*e) * inv_det;

    inv(1,0) = -(d*i - f*g) * inv_det;
    inv(1,1) =  (a*i - c*g) * inv_det;
    inv(1,2) = -(a*f - c*d) * inv_det;

    inv(2,0) =  (d*h - e*g) * inv_det;
    inv(2,1) = -(a*h - b*g) * inv_det;
    inv(2,2) =  (a*e - b*d) * inv_det;

    return inv;
}
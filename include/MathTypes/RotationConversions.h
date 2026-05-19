//
// Created by Ashton Lowenstein on 5/18/26.
//

#ifndef MISSILE6DOF_ROTATIONCONVERSIONS_H
#define MISSILE6DOF_ROTATIONCONVERSIONS_H

#include "MatrixXd.h"
#include "Quaternion.h"

Mat<3,3> toRotationMatrix(Quaternion& q);

Quaternion rotationToQuaternion(const Mat<3,3>& A);

Vec3 toVec3(const Mat<3,1>& A);
Vec3 toVec3(const Mat<1,3>& A);

Mat<3,1> toVecXd(const Vec3& v);

#endif //MISSILE6DOF_ROTATIONCONVERSIONS_H
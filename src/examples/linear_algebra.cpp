//
// Created by Ashton Lowenstein on 5/8/26.
//

#include <iostream>

#include "../include/MathTypes/MatrixXd.h"

int main() {
    Mat<3,3> A = 2 * identity<3>();
    std::cout << isSymmetric(A) << std::endl;
    return 0;
}
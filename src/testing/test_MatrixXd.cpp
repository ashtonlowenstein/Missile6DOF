//
// Created by Ashton Lowenstein on 5/7/26.
//

#include <cassert>
#include <cmath>
#include <stdexcept>
#include "../include/MathTypes/MatrixXd.h"

bool near(double a, double b, double tol = 1e-12) {
    return std::abs(a - b) < tol;
}

void test_default_zero_initialization() {
    Mat<3,4> A{};

    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            assert(near(A(r,c), 0.0));
        }
    }
}

void test_indexing_assignment() {
    Mat<2,3> A{};

    A(0,0) = 1.0;
    A(0,1) = 2.0;
    A(1,2) = 6.0;

    assert(near(A(0,0), 1.0));
    assert(near(A(0,1), 2.0));
    assert(near(A(1,2), 6.0));
}

void test_out_of_bounds() {
    Mat<2,2> A{};

    bool threw = false;
    try {
        A(2,0) = 1.0;
    } catch (const std::out_of_range&) {
        threw = true;
    }

    assert(threw);
}

void test_addition() {
    Mat<2,2> A{};
    Mat<2,2> B{};

    A(0,0) = 1.0; A(0,1) = 2.0;
    A(1,0) = 3.0; A(1,1) = 4.0;

    B(0,0) = 10.0; B(0,1) = 20.0;
    B(1,0) = 30.0; B(1,1) = 40.0;

    auto C = A + B;

    assert(near(C(0,0), 11.0));
    assert(near(C(0,1), 22.0));
    assert(near(C(1,0), 33.0));
    assert(near(C(1,1), 44.0));
}

void test_subtraction() {
    Mat<2,2> A{};
    Mat<2,2> B{};

    A(0,0) = 5.0; A(0,1) = 7.0;
    A(1,0) = 9.0; A(1,1) = 11.0;

    B(0,0) = 1.0; B(0,1) = 2.0;
    B(1,0) = 3.0; B(1,1) = 4.0;

    auto C = A - B;

    assert(near(C(0,0), 4.0));
    assert(near(C(0,1), 5.0));
    assert(near(C(1,0), 6.0));
    assert(near(C(1,1), 7.0));
}

void test_scalar_multiply() {
    Mat<2,2> A{};

    A(0,0) = 1.0; A(0,1) = -2.0;
    A(1,0) = 3.0; A(1,1) = -4.0;

    auto B = A * 2.0;
    auto C = 3.0 * A;

    assert(near(B(0,0), 2.0));
    assert(near(B(0,1), -4.0));
    assert(near(B(1,0), 6.0));
    assert(near(B(1,1), -8.0));

    assert(near(C(0,0), 3.0));
    assert(near(C(0,1), -6.0));
    assert(near(C(1,0), 9.0));
    assert(near(C(1,1), -12.0));
}

void test_matrix_multiply_rectangular() {
    Mat<2,3> A{};
    Mat<3,2> B{};

    A(0,0) = 1.0; A(0,1) = 2.0; A(0,2) = 3.0;
    A(1,0) = 4.0; A(1,1) = 5.0; A(1,2) = 6.0;

    B(0,0) = 7.0;  B(0,1) = 8.0;
    B(1,0) = 9.0;  B(1,1) = 10.0;
    B(2,0) = 11.0; B(2,1) = 12.0;

    auto C = A * B; // Mat<2,2>

    assert(near(C(0,0), 58.0));
    assert(near(C(0,1), 64.0));
    assert(near(C(1,0), 139.0));
    assert(near(C(1,1), 154.0));
}

void test_transpose() {
    Mat<2,3> A{};

    A(0,0) = 1.0; A(0,1) = 2.0; A(0,2) = 3.0;
    A(1,0) = 4.0; A(1,1) = 5.0; A(1,2) = 6.0;

    auto AT = transpose(A); // Mat<3,2>

    assert(near(AT(0,0), 1.0));
    assert(near(AT(1,0), 2.0));
    assert(near(AT(2,0), 3.0));
    assert(near(AT(0,1), 4.0));
    assert(near(AT(1,1), 5.0));
    assert(near(AT(2,1), 6.0));
}

void test_identity() {
    auto I = identity<4>();

    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            if (r == c) {
                assert(near(I(r,c), 1.0));
            } else {
                assert(near(I(r,c), 0.0));
            }
        }
    }
}

void test_solve_single_rhs() {
    Mat<2,2> A{};
    Mat<2,1> b{};

    A(0,0) = 2.0; A(0,1) = 1.0;
    A(1,0) = 1.0; A(1,1) = 3.0;

    b(0,0) = 1.0;
    b(1,0) = 2.0;

    auto x = solve(A, b);

    assert(near(x(0,0), 0.2));
    assert(near(x(1,0), 0.6));
}

void test_solve_multiple_rhs() {
    Mat<2,2> A{};
    Mat<2,2> B{};

    A(0,0) = 2.0; A(0,1) = 1.0;
    A(1,0) = 1.0; A(1,1) = 3.0;

    B(0,0) = 1.0; B(0,1) = 4.0;
    B(1,0) = 2.0; B(1,1) = 5.0;

    auto X = solve(A, B);

    assert(near(X(0,0), 0.2));
    assert(near(X(1,0), 0.6));

    assert(near(X(0,1), 1.4));
    assert(near(X(1,1), 1.2));

    auto AX = A * X;

    assert(near(AX(0,0), B(0,0)));
    assert(near(AX(0,1), B(0,1)));
    assert(near(AX(1,0), B(1,0)));
    assert(near(AX(1,1), B(1,1)));
}

void test_solve_requires_pivoting() {
    Mat<2,2> A{};
    Mat<2,1> b{};

    A(0,0) = 0.0; A(0,1) = 1.0;
    A(1,0) = 2.0; A(1,1) = 3.0;

    b(0,0) = 1.0;
    b(1,0) = 5.0;

    auto x = solve(A, b);

    assert(near(x(0,0), 1.0));
    assert(near(x(1,0), 1.0));
}

void test_solve_singular_throws() {
    Mat<2,2> A{};
    Mat<2,1> b{};

    A(0,0) = 1.0; A(0,1) = 2.0;
    A(1,0) = 2.0; A(1,1) = 4.0;

    b(0,0) = 1.0;
    b(1,0) = 2.0;

    bool threw = false;
    try {
        auto x = solve(A, b);
        (void)x;
    } catch (const std::runtime_error&) {
        threw = true;
    }

    assert(threw);
}

int main() {
    test_default_zero_initialization();
    test_indexing_assignment();
    test_out_of_bounds();

    test_addition();
    test_subtraction();
    test_scalar_multiply();
    test_matrix_multiply_rectangular();

    test_transpose();
    test_identity();

    test_solve_single_rhs();
    test_solve_multiple_rhs();
    test_solve_requires_pivoting();
    test_solve_singular_throws();

    return 0;
}
//
// Created by Ashton Lowenstein on 5/7/26.
//

#ifndef MISSILE6DOF_MATRIXXD_H
#define MISSILE6DOF_MATRIXXD_H
#include <array>
#include <cstddef>
#include <stdexcept>
#include <algorithm>
#include <ostream>
#include <iomanip>
#include "Vec3.h"

template <std::size_t Rows, std::size_t Cols>
class Mat {
public:
    std::array<double, Rows * Cols> data{};

    [[nodiscard]] static std::size_t rows() { return Rows;};
    [[nodiscard]] static std::size_t cols() { return Cols;};

    double& operator()(std::size_t r, std::size_t c) {
        if (r >= Rows || c >= Cols) {
            throw std::out_of_range("Index out of bounds");
        }

        return data[r * Cols + c];
    }

    double operator()(std::size_t r, std::size_t c) const {
        if (r >= Rows || c >= Cols) {
            throw std::out_of_range("Index out of bounds");
        }

        return data[r * Cols + c];
    }

    Mat& operator+=(const Mat& A) {
        for (std::size_t r = 0; r < Rows; r++) {
            for (std::size_t c = 0; c < Cols; c++) {
                (*this)(r, c) += A(r, c);
            }
        }

        return *this;
    }

    Mat& operator-=(const Mat& A) {
        for (std::size_t r = 0; r < Rows; r++) {
            for (std::size_t c = 0; c < Cols; c++) {
                (*this)(r, c) -= A(r, c);
            }
        }

        return *this;
    }

    Mat& operator*=(double x) {
        for (std::size_t r = 0; r < Rows; r++) {
            for (std::size_t c = 0; c < Cols; c++) {
                (*this)(r, c) *= x;
            }
        }

        return *this;
    }

    Mat& fill(double x) {
        for (std::size_t r = 0; r < Rows; r++) {
            for (std::size_t c = 0; c < Cols; c++) {
                (*this)(r, c) = x;
            }
        }

        return *this;
    }

};

// ----------------------------------------------------------------
//                              FUNCTIONS
// ----------------------------------------------------------------

template <std::size_t R, std::size_t C>
Mat<R,C> operator+(const Mat<R,C>& A, const Mat<R,C>& B) {
    Mat<R,C> result{};
    for (std::size_t i = 0; i < R; ++i) {
        for (std::size_t j = 0; j < C; ++j) {
            result(i,j) = A(i,j) + B(i,j);
        }
    }
    return result;
}

template <std::size_t R, std::size_t C>
Mat<R,C> operator-(const Mat<R,C>& A, const Mat<R,C>& B) {
    Mat<R,C> result{};
    for (std::size_t i = 0; i < R; ++i) {
        for (std::size_t j = 0; j < C; ++j) {
            result(i,j) = A(i,j) - B(i,j);
        }
    }
    return result;
}

template <std::size_t R, std::size_t C, std::size_t N>
Mat<R,C> operator*(const Mat<R,N>& A, const Mat<N,C>& B) {
    Mat<R,C> result{};
    for (std::size_t r = 0; r < R; ++r) {
        for (std::size_t c = 0; c < C; ++c) {
            double sum = 0.0;
            for (std::size_t i = 0; i < N; ++i) {
                sum += A(r,i) * B(i,c);
            }
            result(r,c) = sum;
        }
    }
    return result;
}

template <std::size_t R, std::size_t C>
Mat<R,C> operator*(const Mat<R,C>&A, double x) {
    Mat<R,C> result{};
    for (std::size_t i = 0; i < R; i++) {
        for (std::size_t j = 0; j < C; j++) {
            result(i,j) = A(i,j) * x;
        }
    }
    return result;
}

template <std::size_t R, std::size_t C>
Mat<R,C> operator*(double x, const Mat<R,C>&A) {
    return A * x;
}

template <std::size_t R, std::size_t C>
Mat<C,R> transpose(const Mat<R,C>& A) {
    Mat<C,R> result{};

    for (std::size_t i = 0; i < R; ++i) {
        for (std::size_t j = 0; j < C; ++j) {
            result(j,i) = A(i,j);
        }
    }

    return result;
}

template <std::size_t N>
double trace(const Mat<N,N>& A) {
    double result = 0.0;
    for (std::size_t i = 0; i < N; ++i) {
        result += A(i,i);
    }
    return result;
}

template <std::size_t N>
Mat<N,N> identity() {
    Mat<N,N> result{};
    for (std::size_t i = 0; i < N; ++i) {
        result(i,i) = 1.0;
    }

    return result;
}

template <std::size_t R, std::size_t C>
Mat<R,C> zeros() {
    return {};
}

template <std::size_t N>
Mat<N,N> inverse(const Mat<N,N>& A) {
    Mat<N,N> left = A;
    Mat<N,N> right = identity<N>();

    for (std::size_t col = 0; col < N; ++col) {
        // Find pivot
        std::size_t pivot = col;
        double max_abs = std::abs(left(col, col));

        for (std::size_t r = col + 1; r < N; ++r) {
            double value = std::abs(left(r, col));
            if (value > max_abs) {
                max_abs = value;
                pivot = r;
            }
        }

        if (max_abs < 1e-12) {
            throw std::runtime_error("Matrix is singular or ill-conditioned");
        }

        // Swap rows if needed
        if (pivot != col) {
            for (std::size_t c = 0; c < N; ++c) {
                std::swap(left(col, c), left(pivot, c));
                std::swap(right(col, c), right(pivot, c));
            }
        }

        // Normalize pivot row
        double pivot_value = left(col, col);

        for (std::size_t c = 0; c < N; ++c) {
            left(col, c) /= pivot_value;
            right(col, c) /= pivot_value;
        }

        // Eliminate all other rows
        for (std::size_t r = 0; r < N; ++r) {
            if (r == col) continue;

            double factor = left(r, col);

            for (std::size_t c = 0; c < N; ++c) {
                left(r, c) -= factor * left(col, c);
                right(r, c) -= factor * right(col, c);
            }
        }
    }

    return right;
}

template <std::size_t N, std::size_t M>
Mat<N,M> solve(Mat<N,N> A, Mat<N,M> B) {
    constexpr double eps = 1e-12;

    // Forward elimination
    for (std::size_t col = 0; col < N; ++col) {

        // Find pivot row
        std::size_t pivot = col;
        double max_abs = std::abs(A(col, col));

        for (std::size_t r = col + 1; r < N; ++r) {
            double value = std::abs(A(r, col));
            if (value > max_abs) {
                max_abs = value;
                pivot = r;
            }
        }

        if (max_abs < eps) {
            throw std::runtime_error("solve(): matrix is singular or ill-conditioned");
        }

        // Swap rows in A and B
        if (pivot != col) {
            for (std::size_t c = 0; c < N; ++c) {
                std::swap(A(col, c), A(pivot, c));
            }

            for (std::size_t c = 0; c < M; ++c) {
                std::swap(B(col, c), B(pivot, c));
            }
        }

        // Eliminate entries below pivot
        for (std::size_t r = col + 1; r < N; ++r) {
            double factor = A(r, col) / A(col, col);

            A(r, col) = 0.0;

            for (std::size_t c = col + 1; c < N; ++c) {
                A(r, c) -= factor * A(col, c);
            }

            for (std::size_t c = 0; c < M; ++c) {
                B(r, c) -= factor * B(col, c);
            }
        }
    }

    // Back substitution
    Mat<N,M> X{};

    for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
        for (std::size_t rhs = 0; rhs < M; ++rhs) {
            double sum = B(i, rhs);

            for (std::size_t j = i + 1; j < N; ++j) {
                sum -= A(i, j) * X(j, rhs);
            }

            X(i, rhs) = sum / A(i, i);
        }
    }

    return X;
}

template <std::size_t R, std::size_t C>
std::ostream& operator<<(std::ostream& os,
                         const Mat<R,C>& A)
{
    for (std::size_t r = 0; r < R; ++r) {

        os << "[ ";

        for (std::size_t c = 0; c < C; ++c) {
            os << std::setw(6) << A(r,c) << " ";
        }

        os << "]";

        if (r + 1 < R) {
            os << '\n';
        }
    }

    return os;
}

template <std::size_t N>
bool isSymmetric(const Mat<N,N>& A, double tol = 1e-12) {

    for (std::size_t i = 0; i < N; i++) {
        for (std::size_t j = i+1; j < N; j++) {
            if (std::abs(A(i,j) - A(j,i)) > tol) {
                return false;
            }
        }
    }

    return true;
}

#endif //MISSILE6DOF_MATRIXXD_H
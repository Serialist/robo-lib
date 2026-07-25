/**
 * @file demo.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-07-21
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include <iostream>

#include "algorithm/linear-algebra.hpp"
#include "algorithm/quaternion.hpp"

using namespace std;
using algebra::Matrix;
using algebra::Vector;

template<typename T, std::size_t ROW, std::size_t COL>
std::ostream& operator<<(std::ostream& os, const Matrix<T, ROW, COL>& matrix) {
    for (std::size_t i = 0; i < ROW; ++i) {
        for (std::size_t j = 0; j < COL; ++j) {
            os << matrix(i, j) << " ";
        }
        os << std::endl;
    }

    return os;
}

template<typename T, std::size_t SIZE>
std::ostream& operator<<(std::ostream& os, const Vector<T, SIZE>& vector) {
    for (std::size_t i = 0; i < SIZE; ++i) {
        os << vector[i] << " ";
    }

    return os;
}

int main() {
    auto m1 = Matrix<float, 3, 3>::eye();
    Matrix<float, 3, 3> m2 { 0, 1, 0, 0, 0, 1, 1, 0, 0 };
    Vector<float, 3> v1 { 0, 0, 12 };

    auto q1 = algebra::Quaternionf::FromAxisAngle({ 1, 1, 1 }, 3.1415);

    auto v2 = m2 * v1;

    cout << m2.inverse() * v1 << endl;
    cout << v2 << endl;
    cout << v1 * v2 << endl;
    cout << m1 << endl;
    cout << m2 * (v1 + v2) * 3 << endl;

    return 0;
}

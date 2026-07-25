/**
 * @file complex-algebra.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 仿射代数
 * @version 0.1.0
 * @date 2026-07-21
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef COMPLEX_ALGEBRA_HPP
#define COMPLEX_ALGEBRA_HPP

#include <cstdint>

#include "algorithm/linear-algebra.hpp"

namespace algebra {

template<typename T>
class Complex {
private:
    T real;
    T imag;

public:
    Complex(float real, float imag): real(real), imag(imag) {}
    Complex(): real(0), imag(0) {}
    Complex operator+(const Complex& other) const {
        return Complex(real + other.real, imag + other.imag);
    }
    Complex operator-(const Complex& other) const {
        return Complex(real - other.real, imag - other.imag);
    }
};

} // namespace algebra

#endif

/**
 * @file liner-algebra.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 有限维线性空间代数运算库
 * @version 0.1.0
 * @date 2026-06-27
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef LINER_ALGEBRA_HPP
#define LINER_ALGEBRA_HPP

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace algebra {

/* ------------------------------------------------ Vector ------------------------------------------------ */
template<typename T = float, unsigned int N>
class Vector {
private:
    T v[N] {};

public:
    // 默认构造，内置数组值初始化（全部置0）
    Vector() = default;

    // 数组指针构造，explicit 禁止隐式转换
    explicit Vector(const T data[]) {
        std::memcpy(v, data, sizeof(T) * N);
    }

    // 拷贝构造
    Vector(const Vector& other) {
        std::memcpy(v, other.v, sizeof(v));
    }

    // 拷贝赋值运算符
    Vector& operator=(const Vector& other) {
        if (this != &other) {
            std::memcpy(v, other.v, sizeof(v));
        }
        return *this;
    }

    // 可写下标访问（非const对象调用）
    inline T& operator[](unsigned int index) noexcept {
        return v[index];
    }

    // 只读下标访问（const对象调用）
    inline const T& operator[](unsigned int index) const noexcept {
        return v[index];
    }

    // 获取向量维度
    constexpr unsigned int size() const noexcept {
        return N;
    }
};

/* ------------------------------------------------ Matrix ------------------------------------------------ */
template<typename T = float, unsigned int ROW, unsigned int COL>
class Matrix {
private:
    T data[ROW][COL];

public:
    Matrix(T data[ROW][COL]) {
        std::memcpy(this->data, data, sizeof(T) * ROW * COL);
    }

    Matrix() {
        std::memset(this->data, 0, sizeof(T) * ROW * COL);
    }
};

} // namespace algebra

#endif

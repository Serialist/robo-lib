/**
 * @file transform.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-06-20
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <cmath>

namespace rb2 {
namespace algorithm {
namespace transform {

template<typename T = float>
class Quaternion {
public:
    // 构造函数：w, x, y, z
    Quaternion(T w = T(1), T x = T(0), T y = T(0), T z = T(0)) {
        q[0] = w;
        q[1] = x;
        q[2] = y;
        q[3] = z;
    }

    // 拷贝构造
    Quaternion(const Quaternion& other) {
        for (int i = 0; i < 4; ++i)
            q[i] = other.q[i];
    }

    // 数组隐式转换，返回内部数组首地址
    operator T*() {
        return q;
    }
    operator const T*() const {
        return q;
    }

    // 获取 w x y z
    T w() const {
        return q[0];
    }
    T x() const {
        return q[1];
    }
    T y() const {
        return q[2];
    }
    T z() const {
        return q[3];
    }

    // 模长
    T Norm() const {
        return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    }

    // 归一化（原地归一）
    void Normalize() {
        T n = Norm();
        if (n < std::numeric_limits<T>::epsilon())
            return; // 防止除0
        q[0] /= n;
        q[1] /= n;
        q[2] /= n;
        q[3] /= n;
    }

    // 返回归一化后的新四元数，不修改自身
    Quaternion Normalized() const {
        Quaternion res = *this;
        res.Normalize();
        return res;
    }

    // 共轭四元数
    Quaternion Conjugate() const {
        return Quaternion(q[0], -q[1], -q[2], -q[3]);
    }

    // 四元数逆
    Quaternion Inverse() const {
        T n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if (n2 < std::numeric_limits<T>::epsilon())
            return Quaternion {};
        return Conjugate() / n2;
    }

    // 一元负号：返回新四元数，不修改原对象（修复你原来错误写法）
    Quaternion operator-() const {
        return Quaternion(-q[0], -q[1], -q[2], -q[3]);
    }

    // 四元数乘法（成员重载，左乘）
    Quaternion operator*(const Quaternion& other) const {
        const T &w1 = q[0], x1 = q[1], y1 = q[2], z1 = q[3];
        const T &w2 = other.q[0], x2 = other.q[1], y2 = other.q[2], z2 = other.q[3];

        Quaternion res;
        res.q[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        res.q[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        res.q[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        res.q[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
        return res;
    }

    // 四元数加法
    Quaternion operator+(const Quaternion& other) const {
        return Quaternion(
            q[0] + other.q[0],
            q[1] + other.q[1],
            q[2] + other.q[2],
            q[3] + other.q[3]
        );
    }

    // 四元数减法
    Quaternion operator-(const Quaternion& other) const {
        return Quaternion(
            q[0] - other.q[0],
            q[1] - other.q[1],
            q[2] - other.q[2],
            q[3] - other.q[3]
        );
    }

    // 标量乘法
    Quaternion operator*(T scalar) const {
        return Quaternion(q[0] * scalar, q[1] * scalar, q[2] * scalar, q[3] * scalar);
    }

    // 标量除法
    Quaternion operator/(T scalar) const {
        T inv = T(1) / scalar;
        return (*this) * inv;
    }

    // 赋值运算符
    Quaternion& operator=(const Quaternion& other) {
        if (this == &other)
            return *this;
        for (int i = 0; i < 4; ++i)
            q[i] = other.q[i];
        return *this;
    }

    // 点积
    T Dot(const Quaternion& other) const {
        return q[0] * other.q[0] + q[1] * other.q[1] + q[2] * other.q[2] + q[3] * other.q[3];
    }

private:
    T q[4]; // [w, x, y, z]
};

class Rotation {};

template<unsigned int dim = 3, typename T = float>
class Homogeneous {
public:
private:
    T
};

} // namespace transform
} // namespace algorithm
} // namespace rb2

#endif

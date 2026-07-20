/**
 * @file quaternion.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 四元数计算库 —— 旋转变换、插值、姿态解算
 * @version 0.2.0
 * @date 2026-07-20
 *
 * @copyright Copyright (c) Serialist 2026
 *
 * @note 四元数存储顺序: [w, x, y, z]，继承自 std::array<T, 4>
 *       乘法采用 Hamilton 约定 (右手系): i²=j²=k²=ijk=-1
 *       Euler 角采用 ZYX (yaw-pitch-roll) 内旋约定
*/

#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "algorithm/linear-algebra.hpp"

namespace algebra {

using algebra::Vector;

template<typename T>
class Quaternion: public Vector<T, 4> {
private:
    using Base = Vector<T, 4>;

public:
    /// 浮点比较容差
    static constexpr T kEpsilon = T(1e-6);

    /// 编译期 π 常量
    static constexpr T PI() {
        return T(3.14159265358979323846264338327950288419716939937510);
    }

    // -------------------------------- 构造 / 析构 --------------------------------

    /// 默认构造：单位四元数 (1, 0, 0, 0)
    Quaternion(): Base(T(1), T(0), T(0), T(0)) {}

    /// 从 w, x, y, z 分量构造
    Quaternion(T w, T x, T y, T z): Base(w, x, y, z) {}

    /// 从 std::array 构造
    Quaternion(std::array<T, 4> q): Base(q) {}

    /// 从基类 Vector<T, 4> 转换构造
    explicit Quaternion(const Base& v): Base(v) {}

    Quaternion(const Quaternion& q) = default;
    Quaternion(Quaternion&& q) noexcept = default;

    Quaternion& operator=(const Quaternion& q) = default;
    Quaternion& operator=(Quaternion&& q) noexcept = default;

    ~Quaternion() = default;

    // -------------------------------- 元素访问 --------------------------------

    T w() const {
        return (*this)[0];
    }
    T& w() {
        return (*this)[0];
    }
    T x() const {
        return (*this)[1];
    }
    T& x() {
        return (*this)[1];
    }
    T y() const {
        return (*this)[2];
    }
    T& y() {
        return (*this)[2];
    }
    T z() const {
        return (*this)[3];
    }
    T& z() {
        return (*this)[3];
    }

    // -------------------------------- 算术运算符 --------------------------------

    /// 四元数加法
    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(Base::operator+(q));
    }
    /// 四元数减法
    Quaternion operator-(const Quaternion& q) const {
        return Quaternion(Base::operator-(q));
    }
    /// 取负
    Quaternion operator-() const {
        return Quaternion(Base::operator-());
    }

    /// 标量乘法
    Quaternion operator*(T scalar) const {
        return Quaternion(Base::operator*(scalar));
    }
    /// 标量除法
    Quaternion operator/(T scalar) const {
        return Quaternion(Base::operator/(scalar));
    }
    /// 友元：左乘标量
    friend Quaternion operator*(T scalar, const Quaternion& q) {
        return q * scalar;
    }

    Quaternion& operator+=(const Quaternion& q) {
        Base::operator+=(q);
        return *this;
    }
    Quaternion& operator-=(const Quaternion& q) {
        Base::operator-=(q);
        return *this;
    }
    Quaternion& operator*=(T scalar) {
        Base::operator*=(scalar);
        return *this;
    }
    Quaternion& operator/=(T scalar) {
        Base::operator/=(scalar);
        return *this;
    }

    // -------------------------------- 比较 --------------------------------

    /// 相等比较（考虑浮点误差）
    bool operator==(const Quaternion& q) const {
        return Base::operator==(q);
    }
    bool operator!=(const Quaternion& q) const {
        return Base::operator!=(q);
    }

    // -------------------------------- Hamilton 乘积 --------------------------------

    /// 四元数乘法 (Hamilton 乘积)
    /// q1 * q2 表示先施加 q2 旋转，再施加 q1 旋转
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w() * q.w() - x() * q.x() - y() * q.y() - z() * q.z(),
            w() * q.x() + x() * q.w() + y() * q.z() - z() * q.y(),
            w() * q.y() - x() * q.z() + y() * q.w() + z() * q.x(),
            w() * q.z() + x() * q.y() - y() * q.x() + z() * q.w()
        );
    }

    Quaternion& operator*=(const Quaternion& q) {
        return *this = *this * q;
    }

    // -------------------------------- 四元数代数 --------------------------------

    /// 共轭四元数: (w, -x, -y, -z)
    Quaternion Conjugate() const {
        return Quaternion(w(), -x(), -y(), -z());
    }

    /// 四元数的逆: q^{-1} = q* / |q|^2
    /// @note 零范数退化时返回单位四元数
    Quaternion Inverse() const {
        T n2 = this->norm_squared();
        if (n2 < kEpsilon)
            return Quaternion(); // 零范数，返回单位四元数(退化)
        return Conjugate() / n2;
    }

    // -------------------------------- 旋转向量 --------------------------------

    /// 用此四元数旋转三维向量 v
    /// 返回 q * v * q^{-1} (v 作为纯四元数 (0, vx, vy, vz))
    /// @note 使用优化公式避免两次四元数乘法
    Vector<T, 3> Rotate(const Vector<T, 3>& v) const {
        // 优化公式: t = 2·(r × v),  v' = v + w·t + r × t
        // 其中 r = (x, y, z) 是四元数向量部分
        T tx = T(2) * (y() * v[2] - z() * v[1]);
        T ty = T(2) * (z() * v[0] - x() * v[2]);
        T tz = T(2) * (x() * v[1] - y() * v[0]);

        return { v[0] + w() * tx + (y() * tz - z() * ty),
                 v[1] + w() * ty + (z() * tx - x() * tz),
                 v[2] + w() * tz + (x() * ty - y() * tx) };
    }

    // -------------------------------- 从旋转构造 --------------------------------

    /// 从轴角构造: 绕 axis 旋转 angle (弧度)
    /// @param axis  旋转轴 (需已归一化)
    /// @param angle 旋转角度 (弧度)
    static Quaternion FromAxisAngle(const Vector<T, 3>& axis, T angle) {
        T half = angle * T(0.5);
        T s = std::sin(half);
        return Quaternion(std::cos(half), axis[0] * s, axis[1] * s, axis[2] * s);
    }

    /// 从 Euler 角构造 (ZYX 内旋: 先绕 Z 转 yaw, 再绕 Y 转 pitch, 再绕 X 转 roll)
    /// 等价于外旋 XYZ
    /// @param roll  绕 X 轴转角 (弧度)
    /// @param pitch 绕 Y 轴转角 (弧度)
    /// @param yaw   绕 Z 轴转角 (弧度)
    static Quaternion FromEulerAngles(T roll, T pitch, T yaw) {
        T cy = std::cos(yaw * T(0.5));
        T sy = std::sin(yaw * T(0.5));
        T cp = std::cos(pitch * T(0.5));
        T sp = std::sin(pitch * T(0.5));
        T cr = std::cos(roll * T(0.5));
        T sr = std::sin(roll * T(0.5));

        return Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }

    /// 从 3x3 旋转矩阵构造
    /// @param R 行主序 (row-major) 旋转矩阵, R[row][col]
    static Quaternion FromRotationMatrix(const Matrix<T, 3, 3>& R) {
        T trace = R[0][0] + R[1][1] + R[2][2];
        Quaternion q;

        if (trace > T(0)) {
            T s = std::sqrt(trace + T(1)) * T(2); // s = 4*w
            q.w() = s * T(0.25);
            q.x() = (R[2][1] - R[1][2]) / s;
            q.y() = (R[0][2] - R[2][0]) / s;
            q.z() = (R[1][0] - R[0][1]) / s;
        } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
            T s = std::sqrt(T(1) + R[0][0] - R[1][1] - R[2][2]) * T(2); // s = 4*x
            q.w() = (R[2][1] - R[1][2]) / s;
            q.x() = s * T(0.25);
            q.y() = (R[0][1] + R[1][0]) / s;
            q.z() = (R[0][2] + R[2][0]) / s;
        } else if (R[1][1] > R[2][2]) {
            T s = std::sqrt(T(1) + R[1][1] - R[0][0] - R[2][2]) * T(2); // s = 4*y
            q.w() = (R[0][2] - R[2][0]) / s;
            q.x() = (R[0][1] + R[1][0]) / s;
            q.y() = s * T(0.25);
            q.z() = (R[1][2] + R[2][1]) / s;
        } else {
            T s = std::sqrt(T(1) + R[2][2] - R[0][0] - R[1][1]) * T(2); // s = 4*z
            q.w() = (R[1][0] - R[0][1]) / s;
            q.x() = (R[0][2] + R[2][0]) / s;
            q.y() = (R[1][2] + R[2][1]) / s;
            q.z() = s * T(0.25);
        }
        return q.Normalized();
    }

    // -------------------------------- 转换为旋转 --------------------------------

    /// 将四元数转换为轴角表示
    /// @return {axis[3], angle} — 单位旋转轴 + 旋转角 (弧度, [0, π])
    Vector<T, 4> ToAxisAngle() const {
        Quaternion q = this->Normalized();

        T cosHalf = q.w();
        T sinHalf = std::sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());

        Vector<T, 4> axis_angle = { 0, 0, 0, 1 };

        axis_angle[0] = T(2) * std::atan2(sinHalf, cosHalf);

        if (sinHalf > kEpsilon) {
            T inv = T(1) / sinHalf;
            axis[1] = q.x() * inv;
            axis[2] = q.y() * inv;
            axis[3] = q.z() * inv;
        }
        return axis_angle;
    }

    /// 转换为 3x3 旋转矩阵 (行主序)
    Matrix<T, 3, 3> ToRotationMatrix() const {
        T xx = x() * x(), yy = y() * y(), zz = z() * z();
        T xy = x() * y(), xz = x() * z(), yz = y() * z();
        T wx = w() * x(), wy = w() * y(), wz = w() * z();

        return { { { T(1) - T(2) * (yy + zz), T(2) * (xy - wz), T(2) * (xz + wy) },
                   { T(2) * (xy + wz), T(1) - T(2) * (xx + zz), T(2) * (yz - wx) },
                   { T(2) * (xz - wy), T(2) * (yz + wx), T(1) - T(2) * (xx + yy) } } };
    }

    /// 转换为 Euler 角 (ZYX 内旋: roll, pitch, yaw)
    /// @return {roll, pitch, yaw} 单位: 弧度, roll∈[-π,π], pitch∈[-π/2,π/2], yaw∈[-π,π]
    Vector<T, 3> ToEulerAngles() const {
        T roll, pitch, yaw;

        // roll (x-axis rotation)
        T sinr_cosp = T(2) * (w() * x() + y() * z());
        T cosr_cosp = T(1) - T(2) * (x() * x() + y() * y());
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        T sinp = T(2) * (w() * y() - z() * x());
        if (std::fabs(sinp) >= T(1))
            pitch = std::copysign(PI() * T(0.5), sinp); // 万向节死锁：夹紧到 ±90°
        else
            pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        T siny_cosp = T(2) * (w() * z() + x() * y());
        T cosy_cosp = T(1) - T(2) * (y() * y() + z() * z());
        yaw = std::atan2(siny_cosp, cosy_cosp);

        return { roll, pitch, yaw };
    }

    // -------------------------------- 插值 --------------------------------

    /// 球面线性插值 (Slerp)
    /// @param q1 起始四元数
    /// @param q2 终止四元数
    /// @param t  插值参数 [0, 1]
    /// @return 插值结果 (已归一化)
    static Quaternion Slerp(const Quaternion& q1, const Quaternion& q2, T t) {
        T cosOmega = q1.dot(q2);

        // 走最短路径: 若点积为负, 翻转 q2
        Quaternion q2Adj = q2;
        if (cosOmega < T(0)) {
            q2Adj = -q2;
            cosOmega = -cosOmega;
        }

        // 夹角极小 → 线性插值
        if (cosOmega > T(1) - kEpsilon) {
            return Quaternion(
                       q1.w() + t * (q2Adj.w() - q1.w()),
                       q1.x() + t * (q2Adj.x() - q1.x()),
                       q1.y() + t * (q2Adj.y() - q1.y()),
                       q1.z() + t * (q2Adj.z() - q1.z())
            )
                .Normalized();
        }

        T omega = std::acos(cosOmega);
        T sinOmega = std::sin(omega);
        T a = std::sin((T(1) - t) * omega) / sinOmega;
        T b = std::sin(t * omega) / sinOmega;

        return Quaternion(
            a * q1.w() + b * q2Adj.w(),
            a * q1.x() + b * q2Adj.x(),
            a * q1.y() + b * q2Adj.y(),
            a * q1.z() + b * q2Adj.z()
        );
    }

    /// 线性插值后归一化 (Nlerp)
    /// 计算比 Slerp 快，在夹角较小时近似很好
    /// @param q1 起始四元数
    /// @param q2 终止四元数
    /// @param t  插值参数 [0, 1]
    static Quaternion Nlerp(const Quaternion& q1, const Quaternion& q2, T t) {
        // 走最短路径
        Quaternion q2Adj = (q1.dot(q2) < T(0)) ? -q2 : q2;
        Quaternion res = q1 + (q2Adj - q1) * t;
        res.Normalize_self();
        return res;
    }

    // -------------------------------- 指数 / 对数映射 --------------------------------

    /// 四元数的对数映射
    /// log(q) = (0, v * θ) 其中 v = (x,y,z)/|(x,y,z)|, θ = atan2(|v|, w)
    Quaternion Log() const {
        T n = std::sqrt(x() * x() + y() * y() + z() * z());
        T theta = std::atan2(n, w());
        if (n < kEpsilon)
            return Quaternion(T(0), T(0), T(0), T(0));
        T coeff = theta / n;
        return Quaternion(T(0), x() * coeff, y() * coeff, z() * coeff);
    }

    /// 四元数的指数映射
    /// exp(q) = e^w * (cos|v|, sin|v| * v/|v|)  其中 v = (x,y,z)
    Quaternion Exp() const {
        T n = std::sqrt(x() * x() + y() * y() + z() * z());
        T expW = std::exp(w());
        if (n < kEpsilon)
            return Quaternion(expW, T(0), T(0), T(0));
        T coeff = expW * std::sin(n) / n;
        return Quaternion(expW * std::cos(n), x() * coeff, y() * coeff, z() * coeff);
    }

    /// 四元数的 t 次幂 (用于插值)
    /// q^t = exp(t * log(q))
    Quaternion Pow(T t) const {
        return (Log() * t).Exp();
    }

    // -------------------------------- 工具 --------------------------------

    /// 两个四元数间的夹角 (弧度, [0, π])
    static T Angle(const Quaternion& q1, const Quaternion& q2) {
        T dot = q1.dot(q2);
        // 夹紧到 [-1, 1] 防止数值误差
        if (dot > T(1))
            dot = T(1);
        if (dot < T(-1))
            dot = T(-1);
        return T(2) * std::acos(std::fabs(dot));
    }

    /// 单位四元数
    static Quaternion Identity() {
        return Quaternion(T(1), T(0), T(0), T(0));
    }

    /// 判断是否为单位四元数 (模长 ≈ 1)
    bool IsUnit(T tolerance = kEpsilon) const {
        return std::fabs(this->norm_squared() - T(1)) < tolerance;
    }

    /// 取相反符号的四元数 (表示同一旋转)
    Quaternion Opposite() const {
        return -*this;
    }

    // -------------------------------- 基类方法提升 --------------------------------

    /// 原地归一化
    Quaternion& Normalize_self() {
        Base::normalize_self();
        return *this;
    }

    /// 返回归一化后的新四元数，不修改自身
    Quaternion Normalized() const {
        return Quaternion(Base::normalized());
    }

    using Base::angle;
    using Base::distance;
    using Base::dot;
    using Base::norm;
    using Base::norm_squared;
    using Base::project;
};

// -------------------------------- 类型别名 --------------------------------

using Quaternionf = Quaternion<float>;
using Quaterniond = Quaternion<double>;

} // namespace algebra

#endif

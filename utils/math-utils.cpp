/**
 * @file math-utils.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-10
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "math-utils.hpp"

#include <cmath>
#include <cstdint>

using namespace vgd;

// 快速开方（牛顿迭代法）
float math::SSqrt(float x) {
    float y;
    float delta;
    float maxError;

    if (x <= 0) {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine 应该可以改，现在 0.001 是最大相对误差
    maxError = x * 0.001f;

    do {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

/**
 * @brief 快速幂算法 fast power
 *
 * @param a base
 * @param b exponent
 * @return long long
 *
 * @note a ^ b
 */
long long math::FPow(long long a, long long b) {
    long long res = 1;

    while (b) {
        if (b & 1)
            res = res * a;
        a = a * a;
        b >>= 1;
    }
    return res;
}

/**
 * @brief 快速幂取模算法 fast power with mod
 *
 * @param a base
 * @param b exponent
 * @param p mod
 * @return long long
 *
 * @note a ^ b % p
 */
long long math::FPowMod(long long a, long long b, long long p) {
    long long res = 1;

    while (b) {
        if (b & 1)
            res = res * a % p;
        a = a * a % p;
        b >>= 1;
    }
    return res;
}

/// @brief 快速平方根倒数
float math::FiSqrt(float x) {
    float halfnum = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1); // 此处缺 what the fuck（雾）
    y = *(float*)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/// @brief 快速平方根
float math::FSqrtf(float x) {
    float halfnum = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfnum * y * y));
    return 1 / y;
}

/// @brief 最大公约数 greatest common divisor
long long math::FGcd(long long a, long long b) {
    if (b == 0)
        return a;
    return FGcd(b, a % b);
}

/// @brief 限幅
void math::Clampfp(float* in, float min, float max) {
    if (*in < min) {
        *in = min;
    } else if (*in > max) {
        *in = max;
    }
}

/// @brief 限幅
float math::Clampf(float value, float min, float max) {
    return fminf(fmaxf(value, min), max);
}

/// @brief 绝对值限幅
float math::ClampAbsf(float value, float max) {
    return fminf(fmaxf(value, -max), max);
}

float math::Modf(float value, float range) {
    if (range == 0)
        return NAN;
    return value - floorf(value / range) * range;
}

/**
 * @brief 循环限幅函数
 *
 * @param value
 * @param min
 * @param max
 * @return float
 *
 * @note 不做检查，min > max 未定义
 */
float math::LoopClampf(float value, float min, float max) {
    // 实现 1
    // a mod b  ==  a-floor(a/b)*b
    float range = max - min;
    return min + value - floorf(value / range) * range;

    // return min + Modf(value - min, max - min);

    // 实现 2 （注意 -0 问题）
    // 注意 >= 而不是 >，因为符号位可能产生 -0.0 和 0.0
    // -0.0 > 0 是 false，而 0.0 > 0 是 true
    // value = fmodf(value - min, max - min);
    // return (value >= 0) ? (value + min) : (value + max);
}

/// @brief 值映射
float math::Remapf(float a, float inmin, float inmax, float outmin, float outmax) {
    return outmin + (a - inmin) * (outmax - outmin) / (inmax - inmin);
}

/// @brief 斜坡函数
float math::Rampf(float prev_x, float x, float k_min, float k_max, float dt) {
    return x + math::Clampf(prev_x - x, k_min * dt, k_max * dt);
}

// 符号函数
float math::Signf(float value) {
    return (value > 0.0f) - (value < 0.0f);
}

/**
 * @brief 死区函数
 *
 * @param value 输入
 * @param point 死区点
 * @param deadzone 死区大小
 * @return float
 */
float math::Deadzonef(float value, float point, float deadzone) {
    if ((point - deadzone) < value && value < (point + deadzone)) {
        return point;
    } else {
        return value;
    }
}

/// @brief 循环最近距离
/// @param point 当前点（应在 [min, max]）
/// @param setpoint 目标点（应在 [min, max]）
/// @return 循环最近距离
float math::LoopNearestDistance(float point, float setpoint, float min, float max) {
    float drct = setpoint - point;
    float range = max - min;

    if (std::abs(drct) < (range * 0.5)) // 不过零
        return drct;
    else if (math::Sign(drct))
        return drct + range;
    else
        return drct - range;
}

/// @brief 循环最近点
/// @param point 当前点（应在 [min, max]）
/// @param setpoint 目标点（应在 [min, max]）
/// @return 循环最近目标点
float math::LoopNearestPoint(float point, float setpoint, float min, float max) {
    return point + LoopNearestDistance(point, setpoint, min, max);
}

/// @brief 圆圈最近距离
/// @param point 当前点（应在 [0, 2PI]）
/// @param setpoint 目标点（应在 [0, 2PI]）
/// @return 套圈最近距离
float math::CircleNearestDistance(float point, float setpoint) {
    float drct = setpoint - point;
    if (std::abs(drct) < math::pi) // 不过零
        return drct;
    else if (math::Sign(drct))
        return drct + 2 * math::pi;
    else
        return drct - 2 * math::pi;
}

/// @brief 圆圈最近点
/// @param point 当前点（应在 [0, 2PI]）
/// @param setpoint 目标点（应在 [0, 2PI]）
/// @return 套圈最近目标点
/// @note 可用于旋转时解算目标点
float math::CircleNearestPoint(float point, float setpoint) {
    return point + CircleNearestDistance(point, setpoint);
}

/// @brief 半圈最近距离
/// @param point 当前点（应在 [0, PI]）
/// @param setpoint 目标点（应在 [0, PI]）
/// @return 套半圈最近距离
float math::HalfCircleNearestDistance(float point, float setpoint) {
    float drct = setpoint - point;
    if (std::abs(drct) < (math::pi * 0.5)) // 不过零
        return drct;
    else if (math::Sign(drct))
        return drct + math::pi;
    else
        return drct - math::pi;
}

/// @brief 半圈最近点
/// @param point 当前点（应在 [0, PI]）
/// @param setpoint 目标点（应在 [0, PI]）
/// @return 套半圈最近目标点
/// @note 可用于旋转时解算目标点
float math::HalfCircleNearestPoint(float point, float setpoint) {
    return point + CircleNearestDistance(point, setpoint);
}

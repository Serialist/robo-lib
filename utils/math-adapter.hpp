/**
 * @file math-adapter.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 一些数学函数的适配，不同平台移植方便还能硬件优化
 * @version 0.1.0
 * @date 2026-05-11
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef MATH_ADAPTER_HPP
#define MATH_ADAPTER_HPP

#define ARM_MATH
#include "arm_math.h"

namespace rb2 {
namespace math {

#ifdef ARM_MATH

inline float Sin(float x) {
    return arm_sin_f32(x);
}

inline float Cos(float x) {
    return arm_cos_f32(x);
}

inline float Sqrt(float x) {
    float out;
    arm_sqrt_f32(x, &out);
    return out;
}

#else

inline float Sin(float x) {
    return std::sinf(x);
}

inline float Cos(float x) {
    return std::cosf(x);
}

inline float Sqrt(float x) {
    return std::sqrtf(x);
}

#endif

/// @brief 符号函数
/// @param x
/// @return x < 0 true , x >= 0 false
inline bool Sign(float x) {
    return signbit(x);
}

} // namespace math
} // namespace rb2

#endif

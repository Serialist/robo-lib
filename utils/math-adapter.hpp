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

#ifdef ARM_MATH
	#include "arm_math.h"
#endif

namespace vgd {
namespace math {

#ifdef ARM_MATH

inline float sin(float x) {
	return arm_sin_f32(x);
}

inline float cos(float x) {
	return arm_cos_f32(x);
}

inline float sqrt(float x) {
	float out;
	arm_sqrt_f32(x, &out);
	return out;
}

#endif

inline bool sign(float x) {
	return signbit(x);
}

} // namespace math
} // namespace vgd

#endif

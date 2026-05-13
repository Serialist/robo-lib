/**
 * @file math-utils.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-10
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

/* ================================================================ import ================================================================ */

#include "math-adapter.hpp"

/* ================================================================ macro ================================================================ */

/* ================================================================ prototype ================================================================ */

namespace vgd {
namespace math {

constexpr float pi = 3.14159265358979323846; // 圆周率

void Clampfp(float* in, float min, float max);				   // 指针限幅
float Clampf(float value, float min, float max);			   // 限幅
float ClampAbsf(float value, float max);					   // 绝对值限幅
float LoopClampf(float Input, float minValue, float maxValue); // 循环限幅

float Remapf(float a, float inmin, float inmax, float outmin, float outmax); // 值线性映射
float Rampf(float prev_x, float x, float k_min, float k_max, float dt);		 // 斜坡函数
float Deadzonef(float value, float point, float deadzone);					 // 死区

float Bit2Float(int x_int, float x_min, float x_max, int Bits);
int Float2Bit(float x, float x_min, float x_max, int bits);

float Signf(float value);								  // 符号函数
float Modf(float value, float range);					  // 取模
float SSqrt(float x);									  // 开方
long long FPow(long long a, long long b);				  // 快速幂
long long FPowMod(long long a, long long b, long long p); // 快速幂取模
float FiSqrt(float x);									  // 快速平方根倒数
float FSqrtf(float x);									  // 快速平方根
long long FGcd(long long a, long long b);				  // 最大公约数 greatest common divisor

float CircleNearestDistance(float point, float setpoint); // 圈最近目标距离
float CircleNearestPoint(float point, float setpoint);	  // 圈最近目标点

} // namespace math
} // namespace vgd

#endif

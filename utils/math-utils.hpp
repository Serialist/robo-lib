/**
 * @file utils.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef UTILS_HPP
#define UTILS_HPP

/* ================================================================ import ================================================================ */

#include <cmath>
#include <cstdbool>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "arm_math.h"
#include "robo-config.h"

// 引入 blessing
#include "blessing.h"

/* ================================================================ macro ================================================================ */

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#ifndef PI
#define PI 3.14159265354f
#endif

#ifndef EULER_NUMBER
#define EULER_NUMBER 2.718281828f
#endif

#ifndef MIN
#define MIN(a, b) ((a) <= (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) >= (b) ? (a) : (b))
#endif

#define DEG_CLAMPF(Ang)                                                        \
	LoopClampf((Ang), -180.0f, 180.0f)			   // 角度格式化为-180~180
#define RAD_CLAMPF(Ang) LoopClampf((Ang), -PI, PI) // 弧度格式化为-PI~PI

#ifndef DEG2RAD
#define DEG2RAD(Ang) ((Ang) * 0.01745329252f)
#endif

#ifndef RAD2DEG
#define RAD2DEG(Ang) ((Ang) * 57.295779513f)
#endif

#define Angle_to_rad 0.01745329f
#define Rad_to_angle 57.2957732f

#define RadiansToDegrees 57.295779513f
#define DegreesToRadians 0.01745329251f

#define Euler_Number 2.718281828459045f

#ifndef LF
#define LF 0
#define LB 1
#define RF 2
#define RB 3
#endif

// 轮子
#define WL 4
#define WR 5

#define FRONT 0
#define BACK 1

#define LEFT 0
#define RIGHT 1

#ifndef NONE
#define NONE 0
#endif

#ifndef NULL
#define NULL 0
#endif

// 做一层 adapter，方便移植
/// @todo 放到单独一个适配层文件中，比如 math-adapter，这样不同平台移植方便还能硬件优化
#define SINF(x) arm_sin_f32(x)
#define COSF(x) arm_cos_f32(x)

#if defined(BOARD_DM_MC02)

// 用作 buffer，这个是 h7 用的，因为buf需要实时数据不能 cache
#define BUFFER_T __attribute__((section(".AXI_SRAM"))) uint8_t

// cod里用的符号，还没改，先保留下来
#define Matrix arm_matrix_instance_f32
#define Matrix_64 arm_matrix_instance_f64
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32
#define Matrix_Inverse_64 arm_mat_inverse_f64

// 自己定义的符号名，以后改成这个吧
#define MAT32 arm_matrix_instance_f32
#define MAT32_INIT arm_mat_init_f32
#define MAT32_ADD arm_mat_add_f32
#define MAT32_SUB arm_mat_sub_f32
#define MAT32_MULT arm_mat_mult_f32
#define MAT32_TRANS arm_mat_trans_f32
#define MAT32_INVERSE arm_mat_inverse_f32

#elif defined(BOARD_RM_C)

#define BUFFER_T uint8_t

#define MAT32 arm_matrix_instance_f32
#define MAT32_INIT arm_mat_init_f32
#define MAT32_ADD arm_mat_add_f32
#define MAT32_SUB arm_mat_sub_f32
#define MAT32_MULT arm_mat_mult_f32
#define MAT32_TRANS arm_mat_trans_f32
#define MAT32_INVERSE arm_mat_inverse_f32

#endif

/* ================================================================ prototype ================================================================ */

namespace vgd
{

namespace utils
{
void Clampfp(float *in, float min, float max);				   // 指针限幅
float Clampf(float value, float min, float max);			   // 限幅
float ClampAbsf(float value, float max);					   // 绝对值限幅
float LoopClampf(float Input, float minValue, float maxValue); // 循环限幅

float Remapf(
	float a, float inmin, float inmax, float outmin, float outmax); // 值映射
float Rampf(
	float prev_x, float x, float k_min, float k_max, float dt); // 斜坡函数
float Deadzonef(float value, float point, float deadzone);		// 死区

float Bit2Float(int x_int, float x_min, float x_max, int Bits);
int Float2Bit(float x, float x_min, float x_max, int bits);

float Signf(float value);								  // 符号函数
float Modf(float value, float range);					  // 取模
float SSqrt(float x);									  // 开方
long long FPow(long long a, long long b);				  // 快速幂
long long FPowMod(long long a, long long b, long long p); // 快速幂取模
float FiSqrt(float x);									  // 快速平方根倒数
float FSqrtf(float x);									  // 快速平方根
long long FGcd(long long a, long long b); // 最大公约数 greatest common divisor
} // namespace utils

} // namespace vgd

#endif

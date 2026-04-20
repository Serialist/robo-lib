/**
 * @file mecanum.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-13
 *
 * @copyright Copyright (c) Serialist 2026
 *
 * 俯视图：
 *         front
 *  v1 O-----------O v4 <-- 面向轮毂，逆时针为正
 *     |		   |
 *     |    vx     |
 *     |     ^     |
 *     |     |     |
 * left| vy<-+--   |
 *     |     |     |
 *     |     |     |
 *     |	       |
 *     |           |
 *  v2 O-----------O v3
 *
 * vyaw 逆时针为正
 */

#ifndef MECANUM_H
#define MECANUM_H

#include "utils.h"

typedef struct
{
	float vx;	// m/s
	float vy;	// m/s
	float vyaw; // rad/s
	float rw;	// radius_wheel 轮半径 m
	float lx;	// length_x 半车长 m
	float ly;	// length_y 半车宽 m
	float spin; // lx + ly 旋转系数 m
} Mecanum_Chassis_t;

typedef struct
{
	float v1; // 左前
	float v2; // 左后
	float v3; // 右后
	float v4; // 右前
} Mecanum_Wheel_t;

/// @brief 正解
void Mechanum_FK(Mecanum_Wheel_t *w, Mecanum_Chassis_t *ch);

/// @brief 逆解
void Mechanum_IK(Mecanum_Chassis_t *ch, Mecanum_Wheel_t *w);

/// @brief 逆解 easy 版，不考虑轮子位置和单位
void Mechanum_IKez(Mecanum_Chassis_t *ch, Mecanum_Wheel_t *w);

#endif

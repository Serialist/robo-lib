/**
 * @file simple-planner.h
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef SIMPLE_PLANNER_H
#define SIMPLE_PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================ 斜波函数 ================================================================ */

typedef struct {
	float value; // 输出数据
	float kmin; // 斜率最小值
	float kmax; // 斜率最大值
} Ramp_t;

void Ramp_Init(Ramp_t* self, float initial_value, float kmin,
			   float kmax); // 斜波函数初始化
void Ramp_Reset(Ramp_t* self, float value); // 重置
float Ramp_Update(Ramp_t* self, float target, float dt); // 斜波函数计算

#ifdef __cplusplus
}
#endif

#endif

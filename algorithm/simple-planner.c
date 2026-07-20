/**
 * @file simple-planner.c
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "simple-planner.h"

#include "utils/utils.h"

/* ================================================================ 斜波函数 ================================================================ */

/**
 * @brief 斜波函数初始化
 *
 * @param self
 * @param initial_value
 * @param kmin
 * @param kmax
 */
void Ramp_Init(Ramp_t *self, float initial_value, float kmin, float kmax)
{
	self->kmin = kmin;
	self->kmax = kmax;
	self->value = initial_value;
}

void Ramp_Reset(Ramp_t *self, float value)
{
	self->value = value;
}

/**
 * @brief 斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
 *
 * @param self
 * @param target
 * @param dt
 * @return float
 */
float Ramp_Update(Ramp_t *self, float target, float dt)
{
	// 输出增量限幅
	self->value += Clampf(target - self->value, // 增量
						  self->kmin * dt,		// dt 时间内的最大增量
						  self->kmax * dt);
	return self->value;
}

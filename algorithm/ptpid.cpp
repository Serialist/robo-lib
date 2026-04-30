/**
 * @file ptpid.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "ptpid.hpp"

/// @brief
/// @param kp
/// @param ki
/// @param kd
/// @param maxout 最大总输出
/// @param maxintegral 最大积分（不是最大积分输出）
PT::PID::PID(float kp,
			 float ki,
			 float kd,
			 float maxout, //
			 float maxintegral)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->maxout = maxout;
	this->maxintegral = maxintegral;

	this->integral = 0.0f;
	this->prev_error = 0.0f;
	this->out = 0.0f;
}

/// @brief 设置参数
/// @param 同构造函数
void PT::PID::ParamSet(float kp,
					   float ki,
					   float kd,
					   float maxout, //
					   float maxintegral)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->maxout = maxout;
	this->maxintegral = maxintegral;
}

void PT::PID::Reset()
{
	integral = 0.0f;
	prev_error = 0.0f;
	out = 0.0f;
}

float PT::PID::Update(float setpoint, float feedback)
{
	error = setpoint - feedback;

	// 积分计算和限幅
	integral = ClampAbsf(integral + error, maxintegral);

	// 计算
	out = kp * error				   // 比例
		  + ki * integral			   // 积分
		  + kd * (error - prev_error); // 微分

	// 更新反馈值，为下一次差分
	prev_error = error;

	// 输出限幅
	out = ClampAbsf(out, maxout);

	return out;
}

float PT::PID::UpdateEZ(float e, float ie, float de)
{
	out = kp * e + ki * ie + kd * de;
	out = ClampAbsf(out, maxout);
	return out;
}

float PT::PID::ClampAbsf(float x, float max)
{
	if (x > max)
		return max;
	else if (x < -max)
		return -max;
	else
		return x;
}

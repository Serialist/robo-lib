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

pt::PID::PID(float kp, float ki, float kd, float maxout, float maxintegral)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->maxout = maxout;
	this->maxintegral = maxintegral;

	this->integral = 0.0f;
	this->prev_feedback = 0.0f;
	this->out = 0.0f;
}

float pt::PID::Update(float setpoint, float feedback)
{
	// 积分计算和限幅
	integral = ClampAbsf(integral + (setpoint - feedback),
						 maxintegral);

	// 计算
	out = kp * (setpoint - feedback) +
		  ki * integral +
		  kd * (feedback - prev_feedback);

	// 更新反馈值，为下一次差分
	prev_feedback = feedback;

	// 输出限幅
	out = ClampAbsf(out,
					maxout);

	return out;
}

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
vgd::PID::PID(float kp,
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
	this->output = 0.0f;
}

/// @brief 设置参数
/// @param 同构造函数
void vgd::PID::ParamSet(float kp,
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

void vgd::PID::Reset()
{
	integral = 0.0f;
	prev_error = 0.0f;
	output = 0.0f;
}

float vgd::PID::Update(float setpoint, float feedback)
{
	this->setpoint = setpoint;
	this->feedback = feedback;

	// 误差
	error = setpoint - feedback;

	// 积分计算和限幅
	integral = ClampAbsf(integral + error, maxintegral);

	// 微分计算
	derivative = error - prev_error;

	// 计算
	output = kp * error			//
			 + ki * integral	//
			 + kd * derivative; //

	// 输出限幅
	output = ClampAbsf(output, maxout);

	// 更新
	prev_error = error;

	return output;
}

/**
 * @brief 自己赋值的pid，有时候比如de可以是陀螺仪角速度e是角度，这时候可以更准确
 * 
 * @param e 
 * @param ie 
 * @param de 
 * @return float 
*/
float vgd::PID::UpdateEZ(float e, float ie, float de)
{
	error = e;
	integral = ie;
	derivative = de;

	output = kp * e + ki * ie + kd * de;
	output = ClampAbsf(output, maxout);
	return output;
}

float vgd::PID::update_diff(float setpoint, float feedback)
{
	this->setpoint = setpoint;
	this->feedback = feedback;

	// 误差
	error = setpoint - feedback;

	// 计算
	output = kp * (error - prev_error)						// 比例
			 + ki * error									// 积分
			 + kd * (error - 2 * prev_error + pprev_error); // 微分

	// 输出限幅
	output = ClampAbsf(output, maxout);

	// 更新
	pprev_error = prev_error;
	prev_error = error;

	return output;
}

float vgd::PID::ClampAbsf(float x, float max)
{
	if (x > max)
		return max;
	else if (x < -max)
		return -max;
	else
		return x;
}

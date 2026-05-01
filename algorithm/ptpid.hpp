/**
 * @file ptpid.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-25
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef PT_PID_HPP
#define PT_PID_HPP

namespace vgd
{
class PID
{

public:
	float kp, ki, kd;				   // 参数
	float maxout, maxintegral;		   // 限制
	float integral, error, prev_error; // 历史
	float out;						   // 输出

	PID(float kp, float ki, float kd, float maxout, float maxintegral);

	void
	ParamSet(float kp, float ki, float kd, float maxout, float maxintegral);
	void Reset();
	float Update(float setpoint, float feedback);
	float UpdateEZ(float e, float ie, float de);
	float ClampAbsf(float x, float max);
};
} // namespace vgd

#endif

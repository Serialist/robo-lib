/**
 * @file pid.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-21
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef PT_PID_HPP
#define PT_PID_HPP

#include "utils.h"

namespace pt
{
	class PID
	{
	public:
		float kp, ki, kd;			   // 参数
		float maxout, maxintegral;	   // 限制
		float integral, prev_feedback; // 保存历史
		float out;					   // 输出

		PID(float kp, float ki, float kd, float maxout, float maxintegral);

		void ParamSet(float kp, float ki, float kd, float maxout, float maxintegral);
		void Reset();
		float Update(float setpoint, float feedback);
	};
}

#endif

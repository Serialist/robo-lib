/**
 * @file vgd-pid.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-10
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef VGD_PID_HPP
#define VGD_PID_HPP

namespace vgd {
namespace controller {
class PID {
public:
	float kp, ki, kd;								 // 参数
	float maxout, maxintegral;						 // 限制
	float error, prev_error, pprev_error /*上上次*/; // 误差
	float integral;									 // 积分
	float derivative;								 // 微分
	float setpoint, feedback, output;				 // 输入输出
	float dt;

	PID();
	PID(float kp, float ki, float kd, float maxout, float maxintegral);
	PID(float kp, float ki, float kd, float maxout, float maxintegral, float dt);

	void ParamSet(float kp, float ki, float kd, float maxout, float maxintegral);
	void Reset(void);
	float Update(float setpoint, float feedback);
	float Update_Diff(float setpoint, float feedback); // 差分式
	float UpdateEZ(float e, float ie, float de);
	float ClampAbsf(float x, float max);
};
} // namespace controller
} // namespace vgd

#endif

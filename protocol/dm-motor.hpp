/**
 * @file dm-motor.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-08
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef DM_MOTOR_H
#define DM_MOTOR_H

// #include "utils.h"
#include "stdint.h"

namespace vgd {
namespace motor {

class DM {
public:
	uint32_t id;
	uint32_t can_id;
	uint32_t can_master_id;

	enum class Mode : uint32_t { MIT = 0x000, POS = 0x100, SPEED = 0x200 } mode;

	enum class Model {
		DM_H3510,
		DM_4310,
		DM_8009P,
	} model;

	enum class State : uint8_t {
		DISABLE = 0x0,				  // 失能
		ENABLE = 0x1,				  // 使能
		OVER_VOLTAGE = 0x8,			  // 过压
		UNDER_VOLTAGE = 0x9,		  // 欠压
		OVER_CURRENT = 0xA,			  // 过流
		OVER_TEMPERATURE_MOS = 0xB,	  // mos温度过高
		OVER_TEMPERATURE_ROTOR = 0xC, // 电机温度过高
		COMMUNICATION_LOST = 0xD,	  // 通讯丢失
		OVERLOAD = 0xE				  // 过载
	};

	struct Parameter {
		float p_min;
		float p_max;
		float v_min;
		float v_max;
		float kp_min;
		float kp_max;
		float kd_min;
		float kd_max;
		float t_min;
		float t_max;
	} parameter;

	struct Control {
		float kp;
		float kd;

		float position;
		float velocity;

		float torque;
	} control;

	/// @brief 反馈数据
	struct feedback {
		uint32_t id; // 控制器的 ID，取 CAN_ID 的低 8 位
		State state; // 状态

		int position_int; // 位置 int
		int velocity_int; // 速度 int
		int torque_int;	  // 扭矩 int

		float position; // 位置
		float velocity; // 速度
		float torque;	// 扭矩

		uint8_t temperature_mos;   // 驱动上 MOS 的平均温度
		uint8_t temperature_rotor; // 电机内部线圈的平均温度
	} feedback;

	const Parameter dm_4310_param { -12.5f, 12.5f, -30.0f, 30.0f,  0.0f,
									500.0f, 0.0f,  5.0f,   -10.0f, 10.0f };

	DM(uint32_t id, Model model, Mode mode);

	bool Enable();
	bool Disable();
	bool SetZero();
	bool ClearError();

	bool Feedback_Decode(uint8_t* data);
	bool MIT_Encode(float kp, float kd, float position, float velocity, float torque);
	bool PosSpd_Encode(float position, float velocity);
	bool Speed_Encode(float velocity);

	float uint_to_float(int x_int, float x_min, float x_max, int bits);
	int float_to_uint(float x_float, float x_min, float x_max, int bits);
};

} // namespace motor
} // namespace vgd

#endif

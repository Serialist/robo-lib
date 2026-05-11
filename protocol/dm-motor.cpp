/**
 * @file dm-motor.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-08
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "dm-motor.hpp"

namespace vgd
{
namespace motor
{

DM::DM(uint32_t id, Model model, Mode mode)
{
	this->id = id;
	this->model = model;
	this->mode = mode;
	this->can_id = id + (uint32_t)mode;

	switch (model)
	{
	case Model::DM_4310:
		parameter = dm_4310_param;
		break;
	}
}

/* ---------------------------------------------------------------- feedback ---------------------------------------------------------------- */

/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩以及相关温度参数
************************************************************************
**/
bool DM::Feedback_Decode(uint8_t *buf)
{
	feedback.id = buf[0] & 0x0F;
	feedback.state = (State)(buf[0] >> 4);

	feedback.position_int = (buf[1] << 8) | buf[2];
	feedback.velocity_int = (buf[3] << 4) | (buf[4] >> 4);
	feedback.torque_int = ((buf[4] & 0xF) << 8) | buf[5];

	feedback.temperature_mos = (float)(buf[6]);
	feedback.temperature_rotor = (float)(buf[7]);

	feedback.position = uint_to_float(feedback.position_int, parameter.p_min, parameter.p_max, 16); // (-12.5,12.5)
	feedback.velocity = uint_to_float(feedback.velocity_int, parameter.v_min, parameter.v_max, 12); // (-45.0,45.0)
	feedback.torque = uint_to_float(feedback.torque_int, parameter.t_min, parameter.t_max, 12);		// (-18.0,18.0)
}

/* ---------------------------------------------------------------- command ---------------------------------------------------------------- */

/**
************************************************************************
* @brief:      	enable_motor_mode: 启用电机模式函数
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
bool DM::Enable()
{
	uint8_t data[8];

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
}
/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
bool DM::Disable(void)
{
	uint8_t data[8];

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
}
/**
************************************************************************
* @brief:      	save_pos_zero: 保存位置零点函数
* @details:    	通过CAN总线向特定电机发送保存位置零点的命令
************************************************************************
**/
bool DM::SetZero(void)
{
	uint8_t data[8];

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
}
/**
************************************************************************
* @brief:      	clear_err: 清除电机错误函数
* @details:    	通过CAN总线向特定电机发送清除错误的命令。
************************************************************************
**/
bool DM::ClearError(void)
{
	uint8_t data[8];

	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
}

/* ---------------------------------------------------------------- control ---------------------------------------------------------------- */

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
bool DM::MIT_Encode(float kp, float kd, float position, float velocity, float torque)
{
	uint8_t data[8];
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

	pos_tmp = float_to_uint(position, parameter.p_min, parameter.p_max, 16);
	vel_tmp = float_to_uint(velocity, parameter.v_min, parameter.v_max, 12);
	kp_tmp = float_to_uint(kp, parameter.kp_min, parameter.kp_max, 12);
	kd_tmp = float_to_uint(kd, parameter.kd_min, parameter.kd_max, 12);
	tor_tmp = float_to_uint(torque, parameter.t_min, parameter.t_max, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	data[7] = tor_tmp;
}

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   vel:			速度给定值
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
bool DM::PosSpd_Encode(float position, float velocity)
{
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];

	pbuf = (uint8_t *)&position;
	vbuf = (uint8_t *)&velocity;

	data[0] = *pbuf;
	data[1] = *(pbuf + 1);
	data[2] = *(pbuf + 2);
	data[3] = *(pbuf + 3);

	data[4] = *vbuf;
	data[5] = *(vbuf + 1);
	data[6] = *(vbuf + 2);
	data[7] = *(vbuf + 3);
}
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   vel: 			速度给定值
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
bool DM::Speed_Encode(float velocity)
{
	uint8_t *vbuf;
	uint8_t data[8];

	vbuf = (uint8_t *)&velocity;

	data[0] = *vbuf;
	data[1] = *(vbuf + 1);
	data[2] = *(vbuf + 2);
	data[3] = *(vbuf + 3);
}

/* ---------------------------------------------------------------- tool ---------------------------------------------------------------- */

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int DM::float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float DM::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

} // namespace motor
} // namespace vgd

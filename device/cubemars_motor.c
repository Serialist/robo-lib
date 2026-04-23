/**
 * @file cubemars_motor.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2025-12-02
 *
 * @copyright Copyright (c) Serialist 2025
 *
 ************************/

#include "cubemars_motor.h"
#include "bsp-adapter.h"

// servo mode
// void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len)
// {
// 	uint8_t i = 0;
// 	if (len > 8)
// 	{
// 		len = 8;
// 	}
// 	TX_message.StdId = 0;
// 	TX_message.IDE = CAN_ID_EXT;
// 	TX_message.ExtId = id;
// 	TX_message.RTR = CAN_RTR_DATA;
// 	TX_message.DLC = len;
// 	for (i = 0; i < len; i++)
// 	{
// 		buf[i] = data[i];
// 	}
// 	HAL_CAN_AddTxMessage(&hcan1, &TX_message, buf, &send_mailbox2); // CAN 口发送 TxMessage 数据
// }

// void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
// {
// 	buffer[(*index)++] = number >> 24;
// 	buffer[(*index)++] = number >> 16;
// 	buffer[(*index)++] = number >> 8;
// 	buffer[(*index)++] = number;
// }

// void buffer_append_int16(uint8_t *buffer, int16_t number, int16_t *index)
// {
// 	buffer[(*index)++] = number >> 8;
// 	buffer[(*index)++] = number;
// }

// void comm_can_set_current(uint8_t controller_id, float current)
// {
// 	int32_t send_index = 0;
// 	uint8_t buffer[4];
// 	buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &send_index);
// 	comm_can_transmit_eid(controller_id |
// 							  ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
// 						  buffer, send_index);
// }

// void comm_can_set_rpm(uint8_t controller_id, float rpm)
// {
// 	int32_t send_index = 0;
// 	uint8_t buffer[4];
// 	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
// 	comm_can_transmit_eid(controller_id |
// 							  ((uint32_t)CAN_PACKET_SET_RPM << 8),
// 						  buffer, send_index);
// }

// void comm_can_set_pos(uint8_t controller_id, float pos)
// {
// 	int32_t send_index = 0;
// 	uint8_t buffer[4];
// 	buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &send_index);
// 	comm_can_transmit_eid(controller_id |
// 							  ((uint32_t)CAN_PACKET_SET_POS << 8),
// 						  buffer, send_index);
// }

// void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode)
// {
// 	int32_t send_index = 0;
// 	uint8_t buffer;
// 	buffer = set_origin_mode;
// 	comm_can_transmit_eid(controller_id |
// 							  ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8),
// 						  &buffer, send_index);
// }

// motion controller

/**
 * @brief mit 模式使能
 *
 * @param id
 *
 * @note 据说使能命令最好不要重复发送...?
 */
void AK_Motor_MIT_Enable(uint8_t id)
{
	uint8_t buf[8];

	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	buf[4] = 0xFF;
	buf[5] = 0xFF;
	buf[6] = 0xFF;
	buf[7] = 0xFC;

	BSP_CAN_Transmit(BSP_PORT1, id, buf);
}

/**
 * @brief mit 模式失能
 *
 * @param id
 */
void AK_Motor_MIT_Disable(uint8_t id)
{
	uint8_t buf[8];

	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	buf[4] = 0xFF;
	buf[5] = 0xFF;
	buf[6] = 0xFF;
	buf[7] = 0xFD;

	BSP_CAN_Transmit(BSP_PORT1, id, buf);
}

/**
 * @brief 设置编码器零点
 *
 * @param id
 *
 * @note 这个设置的零点位置似乎断电不保存
 */
void AK_Motor_MIT_Setorigin(uint8_t id)
{
	uint8_t buf[8];

	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	buf[4] = 0xFF;
	buf[5] = 0xFF;
	buf[6] = 0xFF;
	buf[7] = 0xFE;

	BSP_CAN_Transmit(BSP_PORT1, id, buf);
}

void AK_Motor_MIT_Control_Encode(float angle, float velocity, float kp, float kd, float torque, uint8_t *buf)
{
	int angle_int,
		velocity_int,
		kp_int,
		kd_int,
		torque_int;

	/// limit data to be within bounds ///

	angle = Clampf(angle, P_MIN, P_MAX);
	velocity = Clampf(velocity, V_MIN, V_MAX);
	kp = Clampf(kp, Kp_MIN, Kp_MAX);
	kd = Clampf(kd, Kd_MIN, Kd_MAX);
	torque = Clampf(torque, T_MIN, T_MAX);

	angle_int = Float2Bit(angle, P_MIN, P_MAX, 16);
	velocity_int = Float2Bit(velocity, V_MIN, V_MAX, 12);
	kp_int = Float2Bit(kp, Kp_MIN, Kp_MAX, 12);
	kd_int = Float2Bit(kd, Kd_MIN, Kd_MAX, 12);
	torque_int = Float2Bit(torque, T_MIN, T_MAX, 12);

	buf[0] = angle_int >> 8;
	buf[1] = angle_int & 0xFF;
	buf[2] = velocity_int >> 4;
	buf[3] = ((velocity_int & 0xF) << 4) | (kp_int >> 8);
	buf[4] = kp_int & 0xFF;
	buf[5] = kd_int >> 4;
	buf[6] = ((kd_int & 0xF) << 4) | (torque_int >> 8);
	buf[7] = torque_int & 0xff;
}

/************************
 * @brief 发送电机 MIT 模式控制
 *
 * @param id 电机 ID
 * @param p_des 目标位置
 * @param v_des 目标速度
 * @param kp 内置 PID 参数
 * @param kd 内置 PID 参数
 * @param t_ff 力矩前馈
 ************************/
void AK_Motor_MIT_Transmit(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff)
{
	uint8_t buf[8];

	AK_Motor_MIT_Control_Encode(p_des, v_des, kp, kd, t_ff, buf);

	BSP_CAN_Transmit(BSP_PORT1, id, buf);
}

void AK_Motor_MIT_Decode(Motor_AK_RxData_t *data, uint8_t buf[8], float pMax, float vMax, float tMax)
{
	data->id = buf[0]; // 驱动 ID 号

	int p_int = (buf[1] << 8) | (buf[2]);
	int v_int = (buf[3] << 4) | (buf[4] >> 4);
	int i_int = ((buf[4] & 0xF) << 8) | (buf[5]);
	int T_int = buf[6];

	float p = Bit2Float(p_int, P_MIN, P_MAX, 16);
	float v = Bit2Float(v_int, V_MIN, V_MAX, 12);
	float i = Bit2Float(i_int, -T_MAX, T_MAX, 12);
	float Temp = T_int;

	data->angle = p;
	data->speed = v;
	data->torque = i;
	data->temp = Temp - 40;
}

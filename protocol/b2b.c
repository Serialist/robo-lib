/**
 * @file b2b.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "b2b.h"

#define SCALE_FACTOR 100

/**
 * @brief 底盘控制命令编码
 *
 * @param vx
 * @param vy
 * @param vyaw
 * @param buf 输出 8 字节
 */
void B2B_Chassis_Command_Encode(B2B_Chassis_Command_t* data, uint8_t* buf) {
	int16_t vx_int = (int16_t)roundf(data->vx * SCALE_FACTOR);
	int16_t vy_int = (int16_t)roundf(data->vy * SCALE_FACTOR);
	int16_t vyaw_int = (int16_t)roundf(data->vyaw * SCALE_FACTOR);

	buf[0] = (vx_int >> 8) & 0xFF;
	buf[1] = vx_int & 0xFF;
	buf[2] = (vy_int >> 8) & 0xFF;
	buf[3] = vy_int & 0xFF;
	buf[4] = (vyaw_int >> 8) & 0xFF;
	buf[5] = vyaw_int & 0xFF;
	buf[6] = (data->sw[0] & 0x03) << 6 | (data->sw[1] & 0x03) << 4;
	buf[7] = (data->button[0] & 0x01) << 7 | (data->button[1] & 0x01) << 6
		| (data->button[2] & 0x01) << 5 | (data->button[3] & 0x01) << 4;
}

/**
 * @brief 底盘控制命令解码
 *
 * @param buf
 * @param vx
 * @param vy
 * @param vyaw
 */
void B2B_Chassis_Command_Decode(uint8_t* buf, B2B_Chassis_Command_t* data) {
	int16_t vx_int = (buf[0] << 8) | buf[1];
	int16_t vy_int = (buf[2] << 8) | buf[3];
	int16_t vyaw_int = (buf[4] << 8) | buf[5];

	data->vx = (float)vx_int / SCALE_FACTOR;
	data->vy = (float)vy_int / SCALE_FACTOR;
	data->vyaw = (float)vyaw_int / SCALE_FACTOR;

	data->sw[0] = (buf[6] >> 6) & 0x03;
	data->sw[1] = (buf[6] >> 4) & 0x03;
	data->button[0] = (buf[7] >> 7) & 0x01;
	data->button[1] = (buf[7] >> 6) & 0x01;
	data->button[2] = (buf[7] >> 5) & 0x01;
	data->button[3] = (buf[7] >> 4) & 0x01;
}

void B2B_Chassis_Feedback_Encode(B2B_Chassis_Feedback_t* data, uint8_t* buf) {
	buf[0] = Float2Bit(data->leglength[0], 0, 0.4, 8);
	buf[1] = Float2Bit(data->leglength[1], 0, 0.4, 8);
	buf[2] = Float2Bit(data->legangle[0], DEG2RAD(-60), DEG2RAD(60), 8);
	buf[3] = Float2Bit(data->legangle[1], DEG2RAD(-60), DEG2RAD(60), 8);
}

void B2B_Chassis_Feedback_Decode(uint8_t* buf, B2B_Chassis_Feedback_t* data) {
	data->leglength[0] = Bit2Float(buf[0], 0, 0.4, 8);
	data->leglength[1] = Bit2Float(buf[1], 0, 0.4, 8);
	data->legangle[0] = Bit2Float(buf[2], DEG2RAD(-60), DEG2RAD(60), 8);
	data->legangle[1] = Bit2Float(buf[3], DEG2RAD(-60), DEG2RAD(60), 8);
}

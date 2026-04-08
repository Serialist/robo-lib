/**
 * @file dji_motor.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-08
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

/* ================================================================ include ================================================================ */

#include "rm_motor.h"

/* ================================================================ define ================================================================ */

/* ================================================================ struct ================================================================ */

/* ================================================================ value ================================================================ */

uint8_t rmmotor_txbuf[8];

/* ================================================================ proto ================================================================ */

/* ================================================================ function ================================================================ */

void RM_Motor_Control_Encode(RM_Motor_Control_t data, uint8_t *buf)
{
	buf[0] = data[0] >> 8;
	buf[1] = data[0];
	buf[2] = data[1] >> 8;
	buf[3] = data[1];
	buf[4] = data[2] >> 8;
	buf[5] = data[2];
	buf[6] = data[3] >> 8;
	buf[7] = data[3];
}

void RM_Motor_Feedback_Decode(uint8_t *buf, RM_Motor_Feedback_t *data)
{
	data->encoder = (int16_t)((buf[0] << 8) | buf[1]);
	data->velocity = (int16_t)((buf[2] << 8) | buf[3]);
	data->current = (int16_t)((buf[4] << 8) | buf[5]);
	data->temperature = buf[6];
}

void RM_Motor_Control_Transmit(CAN_HandleTypeDef *hcan, uint32_t id, RM_Motor_Control_t data)
{
	RM_Motor_Control_Encode(data, rmmotor_txbuf);
	BSP_CAN_Transmit(hcan, id, rmmotor_txbuf);
}

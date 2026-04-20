/**
 * @file rm_motor.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-13
 *
 * @copyright Copyright (c) Serialist 2026
 *
 * @note
 * rad(radian) 弧度 0-2pi
 * deg(degree) 角度 0-360
 * rpm(revolution per minute) 转每分钟
 * rps(revolution per second) 转每秒
 */

#ifndef RM_MOTOR_H
#define RM_MOTOR_H

/* ================================================================ include ================================================================ */

#include "utils.h"
#include "bsp_can.h"

/* ================================================================ define ================================================================ */

// 电机 ID 定义

#define M3508_TX_ID_1 0x200
#define M3508_TX_ID_2 0x1FF
#define M3508_RX_ID(id) (0x200 + (id))

#define M2006_TX_ID_1 0x200
#define M2006_TX_ID_2 0x1FF
#define M2006_RX_ID(id) (0x200 + (id))

#define GM6020_TX_V_ID_1 0x1FF
#define GM6020_TX_V_ID_2 0x2FF
#define GM6020_TX_A_ID_1 0x1FE
#define GM6020_TX_A_ID_2 0x2FE
#define GM6020_RX_ID(id) (0x204 + (id))

#define RM_MOTOR_ANGLE(self) ((float)(self)->encoder / 8191.0f * 2 * PI)   // unit: rad
#define RM_MOTOR_VELOCITY(self) ((float)(self)->velocity / 60.0f * 2 * PI) // unit: rad/s
#define RM_MOTOR_CURRENT(self) ((self)->current)						   // unit: A
#define RM_MOTOR_TEMP(self) ((self)->temperature)						   // unit: C

// 关于 HEXROLL 减速箱 + M3508 电机下的一些转换

#define HEXROLL_VELOCITY(self) ((float)(self)->velocity / 60.0f * 2 * PI / (268.0f / 17.0f)) // unit: rad/s
#define HEXROLL_TORQUE_TO_CURRENT(torque) (int16_t)(torque * 3554.3214161749397f)
#define HEXROLL_CURRENT_TO_TORQUE(curren) ((float)curren * 0.00030059168198529415f)

/* ================================================================ struct ================================================================ */

typedef struct
{
	int16_t encoder;	 // 0-8191 --> 2pi
	int16_t velocity;	 // rpm
	int16_t current;	 // +-16384 --> +-3A
	uint8_t temperature; // C
} RM_Motor_Feedback_t;

/**
 * current +-16384 --> +-3A
 * voltage +-25000 (--> unknown?)
 */
typedef int16_t RM_Motor_Control_t[4];

/* ================================================================ value ================================================================ */

/* ================================================================ proto ================================================================ */

void RM_Motor_Control_Transmit(CAN_HandleTypeDef *hcan, uint32_t id, RM_Motor_Control_t data);

void RM_Motor_Control_Encode(RM_Motor_Control_t data, uint8_t *buf);

void RM_Motor_Feedback_Decode(uint8_t *buf, RM_Motor_Feedback_t *data);

#endif

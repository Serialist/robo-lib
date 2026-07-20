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

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================ include ================================================================ */

#include "utils/bsp-adapter.h"
#include "utils/utils.h"

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

#define M3508_GEAR_RATIO (3591.f / 187.f) // M3508 电机原版减速箱的减速比
#define M2006_GEAR_RATIO (36.0 / 1.0)

// 电机读取数据

#define RM_MOTOR_ANGLE(encoder) ((float)(encoder) * 2.0 * PI / 8191.0) // unit: rad
#define RM_MOTOR_VELOCITY(velocity) ((float)(velocity) / 60 * 2 * PI)  // unit: rad/s
#define RM_MOTOR_CURRENT(self) ((float)(self)->current * 20 / 16384)   // unit: A
#define RM_MOTOR_TEMP(self) ((float)(self)->temperature)               // unit: C

// 关于 HEXROLL 减速箱 + M3508 电机下的一些转换

#define HEXROLL_GEAR_RATIO (268.f / 17.f) // HEXROLL 减速比

#define HEXROLL_VELOCITY(velocity) \
    ((float)velocity / 60.0f * 2 * PI / HEXROLL_GEAR_RATIO) // unit: rad/s

#define HEXROLL_TORQUE_TO_CURRENT(torque) (int16_t)((float)(torque) * 3326.263229308005)
#define HEXROLL_CURRENT_TO_TORQUE(current) (int16_t)((float)(current) / 3326.263229308005)
// approx torque * 3326.263229308005
// #define HEXROLL_TORQUE_TO_CURRENT(torque) \
// 	(int16_t)(torque / HEXROLL_GEAR_RATIO * M3508_GEAR_RATIO / 0.3 / 20 \ * 16384)

// 掉线保护

#define RM_MOTOR_OFFLINE_CNT 100u
#define RM_MOTOR_IS_OFFLINE(self) ((self)->cnt < 50u)
#define RM_MOTOR_HEARTBEAT(self) ((self)->cnt = RM_MOTOR_OFFLINE_CNT)
#define RM_MOTOR_MONITOR(self) \
    { \
        if (!RM_MOTOR_IS_OFFLINE(self)) \
            (self)->cnt--; \
    }

/* ================================================================ struct ================================================================ */

typedef struct {
    int16_t encoder;     // 0-8191 --> 2pi
    int16_t velocity;    // rpm
    int16_t current;     // +-16384 --> +-3A
    uint8_t temperature; // C
    uint8_t cnt;
} RM_Motor_Feedback_t;

/**
	 * current +-16384 --> +-3A
	 * voltage +-25000 (--> unknown?)
	 */
typedef int16_t RM_Motor_Control_t[4];

/* ================================================================ value ================================================================ */

/* ================================================================ proto ================================================================ */

void RM_Motor_Control_Transmit(BSP_Port_t port, uint32_t id, RM_Motor_Control_t data);

void RM_Motor_Control_Encode(RM_Motor_Control_t data, uint8_t* buf);

void RM_Motor_Feedback_Decode(uint8_t* buf, RM_Motor_Feedback_t* data);

#ifdef __cplusplus
}
#endif

#endif

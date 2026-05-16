/**
 * @file b2b.h
 * @author Serialist (ba3pt@qq.com)
 * @brief board to board 板间通信（云底通信）
 * @version 0.1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef B2B_H
#define B2B_H

#ifdef __cplusplus
extern "C" {
#endif

#include "utils.h"

#define B2B_OFFLINE_CNT 100u
#define B2B_IS_OFFLINE(self) ((self)->cnt < 50u)
#define B2B_HEARTBEAT(self) ((self)->cnt = B2B_OFFLINE_CNT)
#define B2B_MONITOR(self) \
    { \
        if (!B2B_IS_OFFLINE(self)) \
            (self)->cnt--; \
    }

typedef struct {
    float ch[4];
    uint8_t sw[4];
    bool btn[8];
    uint8_t cnt;
} B2B_Chassis_Command_t;

typedef struct {
    float leglength[2];
    float legangle[2];
    uint8_t cnt;
} B2B_Chassis_Feedback_t;

void B2B_Chassis_Command_Encode(B2B_Chassis_Command_t* data, uint8_t* buf);
void B2B_Chassis_Command_Decode(uint8_t* buf, B2B_Chassis_Command_t* data);

void B2B_Chassis_Feedback_Encode(B2B_Chassis_Feedback_t* data, uint8_t* buf);
void B2B_Chassis_Feedback_Decode(uint8_t* buf, B2B_Chassis_Feedback_t* data);

#ifdef __cplusplus
}
#endif

#endif

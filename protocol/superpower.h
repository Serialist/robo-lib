/**
 * @file superpower.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-29
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef SUPERPOWER_H
#define SUPERPOWER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "utils.h"

#define SUPERPOWER_FDB_ID 0x211
#define SUPERPOWER_CMD_ID 0x210

typedef struct
{
	float chassis; // 底盘功率
	float cap_tar; // 电容目标功率
	float referee; // 电管输入功率
	float cap;	   // 电容电压
} SuperPower_Fdb_t;

typedef float SuperPower_Cmd_t;

/// @brief 超电反馈解码
/// @param buf
/// @param data
void SuperPower_Fdb_Decode(uint8_t *buf, SuperPower_Fdb_t *data);

/// @brief 超电控制编码
/// @param data
/// @param buf
void SuperPower_Cmd_Encode(float data, uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif

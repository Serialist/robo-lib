/**
 * @file upper-lower-comm.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef UPPER_LOWER_COMM_H
#define UPPER_LOWER_COMM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "utils.h"

typedef struct
{
	float eyaw;
	float epitch;
	float edistance;
	bool is_fire;
} ULC_Control_t;

void ULC_Control_Decode(uint8_t *buf, ULC_Control_t *data);

#ifdef __cplusplus
}
#endif

#endif

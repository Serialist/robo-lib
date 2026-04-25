/**
 * @file upper-lower-comm.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "upper-lower-comm.h"
// #include "byte-convert.h"
#include "comm-tool.h"

#define FRAME_HEAD 0xAA
#define FRAME_TAIL 0x55

void ULC_Control_Decode(uint8_t *buf, ULC_Control_t *data)
{
	if (buf[0] != 0xAA || buf[15] != 0x55)
		return;
	BYTE_TO_FLOAT_4((buf + 1), data->eyaw);
	BYTE_TO_FLOAT_4((buf + 5), data->epitch);
	BYTE_TO_FLOAT_4((buf + 9), data->edistance);
	data->is_fire = buf[13] & 0b00000001;
}

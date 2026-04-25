/**
 * @file comm-tool.c
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-26
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "comm-tool.h"

void Float_To_Byte_4(float *data, uint8_t *buf)
{
#define DATA (uint32_t)*data
	buf[0] = (DATA >> 24) & 0xFF;
	buf[1] = (DATA >> 16) & 0xFF;
	buf[2] = (DATA >> 8) & 0xFF;
	buf[3] = DATA & 0xFF;
#undef DATA
}

void Byte_To_Float_4(uint8_t *buf, float *data)
{
	*data = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

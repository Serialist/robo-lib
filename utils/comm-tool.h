/**
 * @file comm-tool.h
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-26
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef COMM_TOOL_H
#define COMM_TOOL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "utils.h"

#define FLOAT_TO_BYTE_4(f32, pbyte)                                            \
	{                                                                          \
		pbyte[0] = ((uint32_t)f32 >> 24) & 0xFF;                               \
		pbyte[1] = ((uint32_t)f32 >> 16) & 0xFF;                               \
		pbyte[2] = ((uint32_t)f32 >> 8) & 0xFF;                                \
		pbyte[3] = (uint32_t)f32 & 0xFF;                                       \
	}

#define BYTE_TO_FLOAT_4(pbuf, data)                                            \
	(data) = ((pbuf)[0] << 24) | ((pbuf)[1] << 16) | ((pbuf)[2] << 8)          \
			 | ((pbuf)[3]);

#ifdef __cplusplus
}
#endif

#endif

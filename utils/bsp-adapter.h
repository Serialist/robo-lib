/**
 * @file bsp-adapter.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-21
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef BSP_ADAPTER_H
#define BSP_ADAPTER_H

#include "utils.h"

typedef enum
{
	BSP_PORT0 = 0,
	BSP_PORT1,
	BSP_PORT2,
	BSP_PORT3,
	BSP_PORT4,
	BSP_PORT5,
	BSP_PORT6,
	BSP_PORT7,
	BSP_PORT8
} BSP_Port_t;

void BSP_CAN_Transmit(BSP_Port_t port, uint32_t id, uint8_t *data);

#endif

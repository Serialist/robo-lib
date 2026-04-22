/**
 * @file bsp-adapter.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-21
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "bsp-adapter.h"

typedef enum
{
	BSP_ADAPTER_NONE = 0,
	BSP_ADAPTER_ERROR,
	BSP_ADAPTER_OK,
} BSP_Adapter_Status_t;

BSP_Adapter_Status_t bsp_adapter_status;

__weak void BSP_CAN_Transmit(BSP_Port_t port, uint32_t id, uint8_t *data)
{
	bsp_adapter_status = BSP_ADAPTER_ERROR;
	return;
}

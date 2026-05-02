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
#include "main.h"

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
}

/// @note 一种设想中的can接收注册接口，相当于注册listener，不用，感觉过于复杂了，直接在回调里写吧
/// 不过还有一种想法，可以写一个 .h 文件，定义一个#define RX_LIST，在里面写死接收，再让 bspcan调用这个宏。不过其实作为头文件，这样和直接写进去没什么区别吧。
//// __weak void BSP_CAN_AddReceive(BSP_Port_t port, uint32_t id, void (*callback)(uint8_t *buf, void *data), void *data)
// {
// 	bsp_adapter_status = BSP_ADAPTER_ERROR;
// 	return;
// }
// #define BSP_CAN_RECEIVE_LIST_SIZE 10
// typedef struct
// {
// 	uint32_t id;
// 	void (*callback)(uint8_t *buf, void *data);
// 	void *data;
// } BSP_CAN_Receive_t;
// BSP_CAN_Receive_t bsp_can_receive_list[3][BSP_CAN_RECEIVE_LIST_SIZE];
// uint8_t bsp_can_receive_list_index[3] = {0};
// void BSP_CAN_AddReceive(BSP_Port_t port, uint32_t id, void (*callback)(uint8_t *buf, void *data), void *data)
// {
// 	bsp_can_receive_list[port][bsp_can_receive_list_index[port]] =
// 		(BSP_CAN_Receive_t){.id = id,
// 							.callback = callback,
// 							.data = data};
// 	bsp_can_receive_list_index[port]++;
// }
// void BSP_CAN_Listener(BSP_CAN_Receive_t *list, uint32_t id, uint8_t *buf)
// {
// 	for (uint32_t i = 0; i < BSP_CAN_RECEIVE_LIST_SIZE; i++)
// 	{
// 		if (list[i].id == id)
// 		{
// 			list[i].callback(buf, list[i].data);
// 			break;
// 		}
// 	}
// }
// BSP_CAN_Listener(bsp_can_receive_list[BSP_PORT2],
// 				 FDCAN_RxFIFO1Frame.Header.Identifier,
// 				 FDCAN_RxFIFO1Frame.Data);

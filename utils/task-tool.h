/**
 * @file task-tool.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-20
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef TASK_TOOL_H
#define TASK_TOOL_H

#include "cmsis_os.h"
#include "robo-config.h"

/// @brief 按照 freertos.c 写的任务初始化工具宏
// あみのし这个很蠢啊要是再改就建议自己写一遍去 cubemx 生成的 freertos.c 找
//
// Usage:
//
// TASK_DEFINE(CAN_Task, osPriorityHigh, 2048);
//
// void Application_Init(void)
// {
// 		TASK_CREATE(CAN_Task);
// }

#ifndef TASK_DEFINE
/// @brief 任务定义
#define TASK_DEFINE(name, priority, stack_size)            \
	static const osPriority name##_priority = priority;    \
	static uint32_t Start_##name##Buffer[stack_size];      \
	static osStaticThreadDef_t Start_##name##ControlBlock; \
	static osThreadId Start_##name##Handle;                \
	extern void name(void const *argument);
#else
#warning "application.c: TASK_DEFINE macro redefined"
#endif

#ifndef TASK_CREATE
/// @brief 创建任务
#define TASK_CREATE(name)                                                                                                                                            \
	osThreadStaticDef(Start_##name, name, name##_priority, 0, (sizeof(Start_##name##Buffer) / sizeof(uint32_t)), Start_##name##Buffer, &Start_##name##ControlBlock); \
	Start_##name##Handle = osThreadCreate(osThread(Start_##name), NULL);
#else
#warning "application.c: TASK_CREATE macro redefined"
#endif

#endif

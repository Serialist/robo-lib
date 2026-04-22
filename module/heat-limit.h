/**
 * @file heat-limit.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef HEAT_LIMIT_H
#define HEAT_LIMIT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "utils.h"

	float Heat_limit(float cooling_speed, float heat_limit, float heat_per_bullet);

#ifdef __cplusplus
}
#endif

#endif

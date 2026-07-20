/**
 * @file robo-lib.h
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-07-01
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef ROBO_LIB_H
#define ROBO_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

// algorithm
#include "algorithm/MahonyAHRS.h"
#include "algorithm/QuaternionEKF.h"
#include "algorithm/filter.h"
#include "algorithm/fuzzyPID.h"
#include "algorithm/kalman_filter_whx.h"
#include "algorithm/ols-whx.h"
#include "algorithm/pid_whx.h"
#include "algorithm/simple-planner.h"

// device

// module
#include "module/leg-spd.h"
#include "module/lqr.h"
#include "module/mecanum.h"

// protocol
#include "protocol/b2b.h"
#include "protocol/cubemars-motor.h"
#include "protocol/rm_motor.h"
#include "protocol/superpower.h"

// utils
#include "utils/blessing.h"
#include "utils/bsp-adapter.h"
#include "utils/task-tool.h"
#include "utils/utils.h"

#ifdef __cplusplus
}
#endif

#endif

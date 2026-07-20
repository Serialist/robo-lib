/**
 * @file lqr.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-19
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef LQR_H
#define LQR_H

#ifdef __cplusplus
extern "C"
{
#endif

#define U_T 0
#define U_TP 1

void LQR_Control(float *x, float *u, float *u_temp, float len);

#ifdef __cplusplus
}
#endif

#endif

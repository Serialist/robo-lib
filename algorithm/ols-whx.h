/**
 * @file ols-whx.h
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef OLS_WHX_H
#define OLS_WHX_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "utils.h"

/* ================================ OLS Ordinary Least Squares 最小二乘法 ================================ */

typedef
#ifdef BOARD_RM_C
	__packed
#endif
	struct
{
	uint16_t Order;
	uint32_t Count;

	float *x;
	float *y;

	float k;
	float b;

	float StandardDeviation;

	float t[4];
} OLS_t;

void OLS_Init(OLS_t *OLS, uint16_t order);
void OLS_Update(OLS_t *OLS, float deltax, float y);
float OLS_Derivative(OLS_t *OLS, float deltax, float y);
float OLS_Smooth(OLS_t *OLS, float deltax, float y);
float Get_OLS_Derivative(OLS_t *OLS);
float Get_OLS_Smooth(OLS_t *OLS);

#ifdef __cplusplus
}
#endif

#endif

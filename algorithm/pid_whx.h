/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H

#include "user_lib.h"

typedef struct
{
  // PID 参数
  float32_t Kp;
  float32_t Ki;
  float32_t Kd;

  float32_t max_out;  // 最大输出
  float32_t max_iout; // 最大积分输出

  float32_t set;
  float32_t fdb;

  float32_t out;
  float32_t Pout;
  float32_t Iout;
  float32_t Dout;
  float32_t Dbuf[3];  // 微分项 0最新 1上一次 2上上次
  float32_t error[3]; // 误差项 0最新 1上一次 2上上次

  float err_up;
  int cnt_ki;
  int max_cnt_ki;
} PID_Typedef;

void PID_init(PID_Typedef *pid, const float32_t kp, const float32_t ki, const float32_t kd, float32_t max_out, float32_t max_iout);

/// @brief @brief PID 位置式（普通式）
/// @param pid
/// @param set 目标值
/// @param ref 观测值
/// @return
float32_t PID_Update(PID_Typedef *pid, float32_t set, float32_t ref);

/// @brief PID 差分式
/// @param pid
/// @param set 目标值
/// @param ref 观测值
/// @return
float32_t PID_Diff_Update(PID_Typedef *pid, float32_t set, float32_t ref);

/// @brief pid 输出清除
/// @param pid
void PID_Clear(PID_Typedef *pid);

#endif

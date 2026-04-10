/**
 * @file dm_motor.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-06
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "user_lib.h"

#ifndef DEBUG

// /**
//  * @brief typedef enum that contains the type of DJI Motor Device.
//  */
// typedef enum
// {
//   DJI_GM6020,
//   DJI_M3508,
//   DJI_M2006,
//   DJI_MOTOR_TYPE_NUM,
// } DJI_Motor_Type_e;

typedef enum
{
  MIT,
  POSITION_VELOCITY,
  VELOCITY,
} DM_Motor_Control_Mode_Type_e;

typedef enum
{
  Motor_Enable,
  Motor_Disable,
  Motor_Save_Zero_Position,
  DM_Motor_CMD_Type_Num,
} DM_Motor_CMD_Type_e;

/**
 * @brief typedef structure that contains the data for the Motor Device.
 */
typedef struct
{
  bool Initlized;       /*!< init flag */
  int16_t Current;      /*!< Motor electric current */
  int16_t Velocity;     /*!< Motor rotate velocity (RPM)*/
  int16_t Encoder;      /*!< Motor encoder angle */
  int16_t Last_Encoder; /*!< previous Motor encoder angle */
  float Angle;          /*!< Motor angle in degree */
  uint8_t Temperature;  /*!< Motor Temperature */

} DJI_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the param range for the DM_Motor .
 */
typedef struct
{
  float P_MAX;
  float V_MAX;
  float T_MAX;
} DM_Motor_Param_Range_Typedef;

/**
 * @brief typedef structure that contains the data for the DJI Motor Device.
 */
typedef struct
{

  bool Initlized;          /*!< init flag */
  uint8_t State;           /*!< Motor Message */
  uint16_t P_int;          /*!< Motor Positon  uint16 */
  uint16_t V_int;          /*!< Motor Velocity uint16 */
  uint16_t T_int;          /*!< Motor Torque   uint16 */
  float Position;          /*!< Motor Positon  */
  float Velocity;          /*!< Motor Velocity */
  float Torque;            /*!< Motor Torque   */
  float Temperature_MOS;   /*!< Motor Temperature_MOS   */
  float Temperature_Rotor; /*!< Motor Temperature_Rotor */
  float Angle;

} DM_Motor_Data_Typedef;

/**
 * @brief typedef structure that contains the control information for the DM Motor Device .
 */
typedef struct
{
  float Position;
  float Velocity;
  float KP;
  float KD;
  float Torque;
  float Angle;
} DmMotor_Cmd_t;

extern void DM_Motor_Feedback_Decode(DM_Motor_Feedback_t *data, uint8_t *buf);

extern void DM_Motor_Command_Encode(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, DM_Motor_Info_Typedef *DM_Motor, uint8_t CMD);

extern void DM_Motor_Control_Encode(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, DM_Motor_Info_Typedef *DM_Motor, float Postion, float Velocity, float KP, float KD, float Torque);

#endif /* DEBUG */

#endif // DEVICE_MOTOR_H

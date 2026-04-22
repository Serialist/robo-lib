/**
 * @file dm_motor.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-06
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "dm_motor.h"

#if 0

float uint_to_float(int X_int, float X_min, float X_max, int Bits);

int float_to_uint(float x, float x_min, float x_max, int bits);


/**
 * @brief  Transmit enable disable save zero position Command to DM motor
 * @param  *FDCAN_TxFrame£ºpointer to the FDCAN_TxFrame_TypeDef.
 * @param  *DM_Motor£ºpointer to the DM_Motor
 * @param  CMD£ºTransmit Command  (DJI_Motor_Type_e)
 * @retval None
 */
void DmMotor_Cmd_Enable(DM_Motor_CMD_Type_e CMD, uint8_t *buf)
{
	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	buf[4] = 0xFF;
	buf[5] = 0xFF;
	buf[6] = 0xFF;

	switch (CMD)
	{

	case Motor_Enable:
		buf[7] = 0xFC;
		break;

	case Motor_Disable:
		buf[7] = 0xFD;
		break;

	case Motor_Save_Zero_Position:
		buf[7] = 0xFE;
		break;

	default:
		break;
	}
}

/**
 * @brief  CAN Transmit DM motor Information
 * @param  *FDCAN_TxFrame  pointer to the FDCAN_TxFrame_TypeDef.
 * @param  *DM_Motor  pointer to the DM_Motor
 * @param  Postion Velocity KP KD Torgue: Target
 * @retval None
 */
void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, DM_Motor_Info_Typedef *DM_Motor, float Postion, float Velocity, float KP, float KD, float Torque)
{

	if (DM_Motor->Control_Mode == MIT)
	{

		uint16_t Postion_Tmp, Velocity_Tmp, Torque_Tmp, KP_Tmp, KD_Tmp;

		Postion_Tmp = float_to_uint(Postion, -DM_Motor->Param_Range.P_MAX, DM_Motor->Param_Range.P_MAX, 16);
		Velocity_Tmp = float_to_uint(Velocity, -DM_Motor->Param_Range.V_MAX, DM_Motor->Param_Range.V_MAX, 12);
		Torque_Tmp = float_to_uint(Torque, -DM_Motor->Param_Range.T_MAX, DM_Motor->Param_Range.T_MAX, 12);
		KP_Tmp = float_to_uint(KP, 0, 500, 12);
		KD_Tmp = float_to_uint(KD, 0, 5, 12);

		FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier;

		FDCAN_TxFrame->Data[0] = (uint8_t)(Postion_Tmp >> 8);
		FDCAN_TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
		FDCAN_TxFrame->Data[2] = (uint8_t)(Velocity_Tmp >> 4);
		FDCAN_TxFrame->Data[3] = (uint8_t)((Velocity_Tmp & 0x0F) << 4) | (uint8_t)(KP_Tmp >> 8);
		FDCAN_TxFrame->Data[4] = (uint8_t)(KP_Tmp);
		FDCAN_TxFrame->Data[5] = (uint8_t)(KD_Tmp >> 4);
		FDCAN_TxFrame->Data[6] = (uint8_t)((KD_Tmp & 0x0F) << 4) | (uint8_t)(Torque_Tmp >> 8);
		FDCAN_TxFrame->Data[7] = (uint8_t)(Torque_Tmp);
	}
	else if (DM_Motor->Control_Mode == POSITION_VELOCITY)
	{

		uint8_t *Postion_Tmp, *Velocity_Tmp;

		Postion_Tmp = (uint8_t *)&Postion;
		Velocity_Tmp = (uint8_t *)&Velocity;

		FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x100;

		FDCAN_TxFrame->Data[0] = *(Postion_Tmp);
		FDCAN_TxFrame->Data[1] = *(Postion_Tmp + 1);
		FDCAN_TxFrame->Data[2] = *(Postion_Tmp + 2);
		FDCAN_TxFrame->Data[3] = *(Postion_Tmp + 3);
		FDCAN_TxFrame->Data[4] = *(Velocity_Tmp);
		FDCAN_TxFrame->Data[5] = *(Velocity_Tmp + 1);
		FDCAN_TxFrame->Data[6] = *(Velocity_Tmp + 2);
		FDCAN_TxFrame->Data[7] = *(Velocity_Tmp + 3);
	}
	else if (DM_Motor->Control_Mode == VELOCITY)
	{

		uint8_t *Velocity_Tmp;
		Velocity_Tmp = (uint8_t *)&Velocity;

		FDCAN_TxFrame->Header.Identifier = DM_Motor->FDCANFrame.TxIdentifier + 0x200;

		FDCAN_TxFrame->Data[0] = *(Velocity_Tmp);
		FDCAN_TxFrame->Data[1] = *(Velocity_Tmp + 1);
		FDCAN_TxFrame->Data[2] = *(Velocity_Tmp + 2);
		FDCAN_TxFrame->Data[3] = *(Velocity_Tmp + 3);
		FDCAN_TxFrame->Data[4] = 0;
		FDCAN_TxFrame->Data[5] = 0;
		FDCAN_TxFrame->Data[6] = 0;
		FDCAN_TxFrame->Data[7] = 0;
	}

	USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}
//------------------------------------------------------------------------------

/**
 * @brief  Update the DM_Motor Information
 * @param  Identifier:  pointer to the specifies the standard identifier.
 * @param  Rx_Buf:  pointer to the can receive data
 * @param  DM_Motor: pointer to a DM_Motor_Info_Typedef structure that contains the information of DM_Motor
 * @retval None
 */
void DM_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, DM_Motor_Info_Typedef *DM_Motor)
{

	if (*Identifier != DM_Motor->FDCANFrame.RxIdentifier)
		return;

	DM_Motor->Data.State = Rx_Buf[0] >> 4;
	DM_Motor->Data.P_int = ((uint16_t)(Rx_Buf[1]) << 8) | ((uint16_t)(Rx_Buf[2]));
	DM_Motor->Data.V_int = ((uint16_t)(Rx_Buf[3]) << 4) | ((uint16_t)(Rx_Buf[4]) >> 4);
	DM_Motor->Data.T_int = ((uint16_t)(Rx_Buf[4] & 0xF) << 8) | ((uint16_t)(Rx_Buf[5]));
	DM_Motor->Data.Torque = uint_to_float(DM_Motor->Data.T_int, -DM_Motor->Param_Range.T_MAX, DM_Motor->Param_Range.T_MAX, 12);
	DM_Motor->Data.Position = uint_to_float(DM_Motor->Data.P_int, -DM_Motor->Param_Range.P_MAX, DM_Motor->Param_Range.P_MAX, 16);
	DM_Motor->Data.Velocity = uint_to_float(DM_Motor->Data.V_int, -DM_Motor->Param_Range.V_MAX, DM_Motor->Param_Range.V_MAX, 12);

	DM_Motor->Data.Temperature_MOS = (float)(Rx_Buf[6]);
	DM_Motor->Data.Temperature_Rotor = (float)(Rx_Buf[7]);
}
//------------------------------------------------------------------------------

static float uint_to_float(int X_int, float X_min, float X_max, int Bits)
{
	float span = X_max - X_min;
	float offset = X_min;
	return ((float)X_int) * span / ((float)((1 << Bits) - 1)) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits)
{

	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

#endif

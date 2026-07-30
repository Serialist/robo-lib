/**
 * @file b2b-omnipie.c
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-07-30
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "b2b-omnipie.h"

#include <math.h>
#include <stdint.h>

// FDCAN_TxFrame->Header.Identifier = 0x21;
void B2B_OmniPie_CtrlMsg_Encode(struct B2B_OmniPie_Data* data, uint8_t* buf) {
    int16_t data_int[4];

    data_int[0] = data->vx_set * 1000;   // vx
    data_int[1] = data->vy_set * 1000;   // vy
    data_int[2] = data->vyaw_set * 1000; // vyaw
    data_int[3] = data->reset_flag;

    buf[0] = (uint8_t)(data_int[0] >> 8);
    buf[1] = (uint8_t)data_int[0];
    buf[2] = (uint8_t)(data_int[1] >> 8);
    buf[3] = (uint8_t)data_int[1];
    buf[4] = (uint8_t)(data_int[2] >> 8);
    buf[5] = (uint8_t)data_int[2];
    buf[6] = (uint8_t)(data_int[3] >> 8);
    buf[7] = (uint8_t)data_int[3];
};

// debug var for UI
int16_t debug_ui[4];

// FDCAN_TxFrame->Header.Identifier = 0x22; // 与接收方匹配的ID
void B2B_OmniPie_InfoMsg_Encode(struct B2B_OmniPie_Data* data, uint8_t* buf) {
    // data for UI and yaw motor
    int16_t data_int[4] = { 0 }; // 初始化所有元素为0

    // data[0]：集中存放所有标志位
    {
        // 小陀螺
        if (data->spinbot) {
            data_int[0] |= (1 << 0);
        }
        // 正常
        if (data->nospinbot) {
            data_int[0] |= (1 << 1);
        }
        // 无力
        if (data->zero_force) {
            data_int[0] |= (1 << 2);
        }
        // 开摩擦轮
        if (data->fricWheel_enable) {
            data_int[0] |= (1 << 3);
        }
        // 发射
        if (data->fire_enable) {
            data_int[0] |= (1 << 4);
        }
        // 识别到
        if (data->aimbot_seen) {
            data_int[0] |= (1 << 5);
        }
        // 火控开
        if (data->aimbot_fire) {
            data_int[0] |= (1 << 6);
        }
        // 重启
        if (data->reset_flag) {
            data_int[0] |= (1 << 7);
        }
    }

    // data[1]：pitch angle
    data_int[1] = (int16_t)(data->pitch_angle * 100.0f);

    // data[2]：保留原有的电流值
    data_int[2] = data->shoot_current_set;

    // data[3]：整型保留位
    data_int[3] = 0; // 留作未来扩展使用

    // 大端格式
    buf[0] = (uint8_t)(data_int[0] >> 8);
    buf[1] = (uint8_t)data_int[0];
    buf[2] = (uint8_t)(data_int[1] >> 8);
    buf[3] = (uint8_t)data_int[1];
    buf[4] = (uint8_t)(data_int[2] >> 8);
    buf[5] = (uint8_t)data_int[2];
    buf[6] = (uint8_t)(data_int[3] >> 8);
    buf[7] = (uint8_t)data_int[3];

    // 更新监视数组
    debug_ui[0] = data_int[0];
    debug_ui[1] = data_int[1];
    debug_ui[2] = data_int[2];
    debug_ui[3] = data_int[3];
}

// id 0x23
void B2B_OmniPie_RefMsg_Decode(uint8_t* buf, struct B2B_OmniPie_Data* data) {
    int16_t speed_raw = (buf[0] << 8) | buf[1];
    data->initial_speed = (float)speed_raw / 100.0f;
    data->shooter_barrel_heat = (buf[2] << 8) | buf[3];
    data->shooter_barrel_heat_limit = (buf[4] << 8) | buf[5];
    data->shooter_barrel_cooling_value = (buf[6] << 8) | buf[7];
}

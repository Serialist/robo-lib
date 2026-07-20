/**
 * @file superpower.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-29
 *
 * @copyright Copyright (c) Serialist 2026
 *
 * 参考 [【RM】VGD25超电CAN通信文档](https://hongliu.icu/2025/09/SuperCap-CAN/)
 * P_referee = P_chassis + P_cap
 */

#include "superpower.h"

/**
 * @brief 超级电容反馈信息解码
 *
 * @param buf
 * @param data
 */
void SuperPower_Fdb_Decode(uint8_t* buf, SuperPower_Fdb_t* data) {
    uint16_t chassis = (buf[1] << 8 | buf[0]);
    uint16_t cap_tar = (buf[3] << 8 | buf[2]);
    uint16_t referee = (buf[5] << 8 | buf[4]);
    uint16_t cap = (buf[7] << 8 | buf[6]);

    data->chassis = (float)(chassis - 0x7FFF) / 10.0f;
    data->cap_tar = (float)(cap_tar - 0x7FFF) / 10.0f;
    data->referee = (float)(referee - 0x7FFF) / 10.0f;
    data->cap = (float)cap / 100.0f;
    data->cnt = 100u;
    data->isOnline = true;
}

void SuperPower_Monitor(SuperPower_Fdb_t* self) {
    if (self->cnt < 50u) {
        self->isOnline = false;
    } else {
        self->cnt--;
    }
}

/**
 * @brief 超级电容控制命令编码
 *
 * @param data
 * @param buf
 *
 * @note buf[2..7] 保留位
 */
void SuperPower_Cmd_Encode(float data, uint8_t* buf) {
    uint16_t data_mapped = data * 10 + 0x7FFF;

    buf[0] = data_mapped;
    buf[1] = data_mapped >> 8;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

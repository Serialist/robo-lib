/************************************************
 * @file fsi6.cpp
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief fsi6 协议解析
 * @version 1.0.0
 * @date 2025-10-04
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 * @note
 * 这是富斯遥控器 FSI6 协议的解析代码，根据以下的官网信息编写。
 * [富斯AFHDS3 FSI6 协议通道数据格式公布公告](https://www.flyskytech.com/info_detail/18.html)
 *
 * 数据格式如下：
 * 串口格式：Baud Rate 115200 (8, N, 1)
 * 帧长：32 bytes
 ********************************/

#include "fsi6.hpp"

/// @brief 通道信息
#define FSI6_CH_LEN 18 // 通道数

/// @brief 帧信息
#define FSI6_FARME_LEN 32
#define FSI6_FARME_HEAD_0 0x20
#define FSI6_FARME_HEAD_1 0x40

/// @brief 通道映射
// 摇杆
#define FSI6_CH_JOY_LX 4
#define FSI6_CH_JOY_LY 3
#define FSI6_CH_JOY_RX 1
#define FSI6_CH_JOY_RY 2
// 波轮
#define FSI6_CH_VR_A 5
#define FSI6_CH_VR_B 6
// 开关
#define FSI6_CH_SW_A 6
#define FSI6_CH_SW_B 7
#define FSI6_CH_SW_C 8
#define FSI6_CH_SW_D 9

void FSi6::Parse(uint8_t* buf, uint8_t len) {
    uint8_t i = 0;
    uint8_t t; // temp index

    /* ================================ 校验 ================================ */

    if (len != FSI6_FARME_LEN          // 帧长不对
        || buf[0] != FSI6_FARME_HEAD_0 // 帧头不对
        || buf[1] != FSI6_FARME_HEAD_1
        || Checksum8(buf, FSI6_FARME_LEN - 2) != (buf[30] | (buf[31] << 8))) // 校验和不对
    {
        return;
    }

    /* ================================ 解析 ================================ */
    // ch 1 -- 14
    for (; i < 14; i++) {
        t = 2 * i;
        channel[i] = (uint16_t)((buf[t + 2] | (buf[t + 3] << 8)) & 0x07FF);
    }
    // ch 15 -- 18
    for (; i < 18; i++) {
        t = (i - 14) * 6;
        channel[i] = (uint16_t)(((buf[t + 3] >> 4) | buf[t + 5] | (buf[t + 7] << 4)) & 0x07FF);
    }

    /* ================================ 处理 ================================ */

    joy[RC_LX] = channel[FSI6_CH_JOY_LX] - FSI6_CH_MID;
    joy[RC_LY] = channel[FSI6_CH_JOY_LY] - FSI6_CH_MID;
    joy[RC_RX] = channel[FSI6_CH_JOY_RX] - FSI6_CH_MID;
    joy[RC_RY] = channel[FSI6_CH_JOY_RY] - FSI6_CH_MID;

    vr[RC_A] = channel[FSI6_CH_VR_A] - FSI6_CH_MID;
    vr[RC_A] = channel[FSI6_CH_VR_B] - FSI6_CH_MID;

    sw[RC_A] = channel[FSI6_CH_SW_A];
    sw[RC_B] = channel[FSI6_CH_SW_B];
    sw[RC_C] = channel[FSI6_CH_SW_C];
    sw[RC_D] = channel[FSI6_CH_SW_D];
}

/// @brief 计算校验和
/// @note checksum = sum(byte[0:len]) ^ 0xFFFF
uint8_t FSi6::Checksum8(const uint8_t* data, uint8_t len) {
    uint16_t checksum = 0;

    for (uint8_t i = 0; i < len; i++) {
        checksum += data[i];
    }

    return checksum ^ 0xFFFF;
}

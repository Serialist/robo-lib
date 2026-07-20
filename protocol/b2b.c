/**
 * @file b2b.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "b2b.h"
#include "utils/utils.h"

#define SCALE_FACTOR 100

#define CH_MIN_0 -PI
#define CH_MAX_0 PI
#define CH_MIN_1 0
#define CH_MAX_1 1

/**
 * @brief 底盘控制命令编码
 *
 * @param vx
 * @param vy
 * @param vyaw
 * @param buf 输出 8 字节
 */
void B2B_Chassis_Command_Encode(B2B_Chassis_Command_t* data, uint8_t* buf) {
    int16_t ch[4];

    // 通道映射
    ch[0] = Float2Bit(data->ch[0], CH_MIN_0, CH_MAX_0, 12);
    ch[1] = Float2Bit(data->ch[1], CH_MIN_0, CH_MAX_0, 12);
    ch[2] = Float2Bit(data->ch[2], CH_MIN_0, CH_MAX_0, 12);
    ch[3] = Float2Bit(data->ch[3], CH_MIN_1, CH_MAX_1, 12);

    // 12 位编码
    buf[0] = (ch[0] >> 4) & 0xFF;
    buf[1] = ((ch[0] << 4) & 0xF0) | ((ch[1] >> 8) & 0x0F);
    buf[2] = ch[1] & 0xFF;
    buf[3] = (ch[2] >> 4) & 0xFF;
    buf[4] = ((ch[2] << 4) & 0xF0) | ((ch[3] >> 8) & 0x0F);
    buf[5] = ch[3] & 0xFF;

    // 开关
    buf[6] = (data->sw[0] & 0x03) << 6 | (data->sw[1] & 0x03) << 4 //
        | (data->sw[2] & 0x03) << 2 | (data->sw[3] & 0x03);

    // 按键
    buf[7] = (data->btn[0] & 0x01) << 7 | (data->btn[1] & 0x01) << 6 //
        | (data->btn[2] & 0x01) << 5 | (data->btn[3] & 0x01) << 4    //
        | (data->btn[4] & 0x01) << 3 | (data->btn[5] & 0x01) << 2    //
        | (data->btn[6] & 0x01) << 1 | (data->btn[7] & 0x01);
}

/**
 * @brief 底盘控制命令解码
 *
 * @param buf
 * @param vx
 * @param vy
 * @param vyaw
 */
void B2B_Chassis_Command_Decode(uint8_t* buf, B2B_Chassis_Command_t* data) {
    int16_t ch[4];

    // 通道值解码
    data->ch[0] =
        Bit2Float(((buf[0] << 4) & 0xFF0) | ((buf[1] >> 4) & 0x00F), CH_MIN_0, CH_MAX_0, 12);
    data->ch[1] = Bit2Float(((buf[1] << 8) & 0xF00) | (buf[2] & 0xFF), CH_MIN_0, CH_MAX_0, 12);
    data->ch[2] =
        Bit2Float(((buf[3] << 4) & 0xFF0) | ((buf[4] >> 4) & 0x00F), CH_MIN_0, CH_MAX_0, 12);
    data->ch[3] = Bit2Float(((buf[4] << 8) & 0xF00) | (buf[5] & 0xFF), CH_MIN_1, CH_MAX_1, 12);

    // 开关
    data->sw[0] = (buf[6] >> 6) & 0x03;
    data->sw[1] = (buf[6] >> 4) & 0x03;
    data->sw[2] = (buf[6] >> 2) & 0x03;
    data->sw[3] = buf[6] & 0x03;

    // 按键
    data->btn[0] = (buf[7] >> 7) & 0x01;
    data->btn[1] = (buf[7] >> 6) & 0x01;
    data->btn[2] = (buf[7] >> 5) & 0x01;
    data->btn[3] = (buf[7] >> 4) & 0x01;
    data->btn[4] = (buf[7] >> 3) & 0x01;
    data->btn[5] = (buf[7] >> 2) & 0x01;
    data->btn[6] = (buf[7] >> 1) & 0x01;
    data->btn[7] = buf[7] & 0x01;

    B2B_HEARTBEAT(data);
}

void B2B_Chassis_Feedback_Encode(B2B_Chassis_Feedback_t* data, uint8_t* buf) {
    buf[0] = Float2Bit(data->leglength[0], 0, 0.4, 8);
    buf[1] = Float2Bit(data->leglength[1], 0, 0.4, 8);
    buf[2] = Float2Bit(data->legangle[0], DEG2RAD(-60), DEG2RAD(60), 8);
    buf[3] = Float2Bit(data->legangle[1], DEG2RAD(-60), DEG2RAD(60), 8);
}

void B2B_Chassis_Feedback_Decode(uint8_t* buf, B2B_Chassis_Feedback_t* data) {
    data->leglength[0] = Bit2Float(buf[0], 0, 0.4, 8);
    data->leglength[1] = Bit2Float(buf[1], 0, 0.4, 8);
    data->legangle[0] = Bit2Float(buf[2], DEG2RAD(-60), DEG2RAD(60), 8);
    data->legangle[1] = Bit2Float(buf[3], DEG2RAD(-60), DEG2RAD(60), 8);

    B2B_HEARTBEAT(data);
}

/**
 * @file rm-motor.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-18
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

/* ================================================================ include ================================================================ */

#include "rm-motor.hpp"

namespace rb2 {
namespace device {

class RM_Motor {
    void Encode(Control data) {
        buf[0] = data[0] >> 8;
        buf[1] = data[0];
        buf[2] = data[1] >> 8;
        buf[3] = data[1];
        buf[4] = data[2] >> 8;
        buf[5] = data[2];
        buf[6] = data[3] >> 8;
        buf[7] = data[3];
    }

    void Decode(uint8_t* buf) {
        feedback.encoder = (int16_t)((buf[0] << 8) | buf[1]);
        feedback.velocity = (int16_t)((buf[2] << 8) | buf[3]);
        feedback.current = (int16_t)((buf[4] << 8) | buf[5]);
        feedback.temperature = buf[6];
        // RM_MOTOR_HEARTBEAT(data);

        feedback.cnt = 0;
    }

    void Transmit(Control data) {
        Encode(data);
        // BSP_CAN_Transmit(port, id, rmmotor_txbuf);
    }

     void Monitor(void) {
        if (feedback.cnt > 50) {
            isOnline = false;
        } else {
            feedback.cnt++;
        }
    };

} // namespace device
} // namespace rb2

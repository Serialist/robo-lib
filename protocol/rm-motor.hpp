/**
 * @file rm-motor.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-18
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef RM_MOTOR_HPP
#define RM_MOTOR_HPP

#include "utils.h"
#include <stdint.h>

namespace rb2 {
namespace device {

class RM_Motor {
private:
public:
    // 电机 ID 定义

#define M3508_TX_ID_1 0x200
#define M3508_TX_ID_2 0x1FF
#define M3508_RX_ID(id) (0x200 + (id))

#define M2006_TX_ID_1 0x200
#define M2006_TX_ID_2 0x1FF
#define M2006_RX_ID(id) (0x200 + (id))

#define GM6020_TX_V_ID_1 0x1FF
#define GM6020_TX_V_ID_2 0x2FF
#define GM6020_TX_A_ID_1 0x1FE
#define GM6020_TX_A_ID_2 0x2FE
#define GM6020_RX_ID(id) (0x204 + (id))

#define M3508_GEAR_RATIO (3591.f / 187.f) // M3508 电机原版减速箱的减速比
#define M2006_GEAR_RATIO (36 / 1)

    // 电机读取数据

#define RM_MOTOR_ANGLE(encoder) ((float)encoder * 2.0 * PI / 8191.0)    // unit: rad
#define RM_MOTOR_VELOCITY(self) ((float)(self)->velocity / 60 * 2 * PI) // unit: rad/s
#define RM_MOTOR_CURRENT(self) ((float)(self)->current * 20 / 16384)    // unit: A
#define RM_MOTOR_TEMP(self) ((float)(self)->temperature)                // unit: C

    // 关于 HEXROLL 减速箱 + M3508 电机下的一些转换

#define HEXROLL_GEAR_RATIO (268.f / 17.f) // HEXROLL 减速比

#define HEXROLL_VELOCITY(self) \
    ((float)(self)->velocity / 60.0f * 2 * PI / HEXROLL_GEAR_RATIO) // unit: rad/s

#define HEXROLL_TORQUE_TO_CURRENT(torque) (int16_t)(torque * 3326.263229308005)
#define HEXROLL_CURRENT_TO_TORQUE(current) (int16_t)(current / 3326.263229308005)
    // approx torque * 3326.263229308005
    // #define HEXROLL_TORQUE_TO_CURRENT(torque) \
// 	(int16_t)(torque / HEXROLL_GEAR_RATIO * M3508_GEAR_RATIO / 0.3 / 20 \ * 16384)

    // 掉线保护

    using ID = uint8_t;

    struct Feedback {
        int16_t encoder;     // 0-8191 --> 2pi
        int16_t velocity;    // rpm
        int16_t current;     // +-16384 --> +-3A
        uint8_t temperature; // C
        uint8_t cnt;
    } feedback;

    enum class Type {
        M3508,
        M2006,
        GM6020,
        HEXROLL,
    } type;

    BUFFER_T buf[8];

    /**
	 * current +-16384 --> +-3A
	 * voltage +-25000 (--> unknown?)
	 */
    typedef int16_t Control[4];

    RM_Motor(BSP::Port port, Type type, ID id);

    void Transmit(int16_t current[4]);
    inline void Transmit(int16_t current1, int16_t current2, int16_t current3, int16_t current4) {
        Transmit((int16_t[4]) { current1, current2, current3, current4 });
    }

    void Receive(uint8_t* buf);

    inline int16_t GetEncoder(void) {
        return feedback.encoder;
    }
    inline float GetAngle(void) {
        ` return RM_MOTOR_ANGLE(feedback.encoder);
    }
    inline float GetVelocity_RPM(void) {
        return RM_MOTOR_VELOCITY(&feedback);
    }

    inline float GetVelocity(void) {
        return feedback.velocity;
        0
    }

    inline float GetCurrent(void) {
        return RM_MOTOR_CURRENT(&feedback);
    }
    inline float GetTemperature(void) {
        return RM_MOTOR_TEMP(&feedback);
    }

    void Control_Encode(Control data);

    void Decode(uint8_t* buf);
};

} // namespace device
} // namespace rb2

#endif

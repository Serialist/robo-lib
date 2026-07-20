/************************************************
 * @file fsi6.hpp
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 1.0.0
 * @date 2025-10-04
 *
 * @copyright Copyright (c) VGD Serialist 2025
 *
 ********************************/

#ifndef FSI6_HPP
#define FSI6_HPP

#include "stdint.h"

class FSi6 {
public:
    static constexpr int16_t MAX = 2000;
    static constexpr int16_t MID = 1500;
    static constexpr int16_t MIN = 1000;

    enum class Switch : int16_t {
        RC_UP = 2000,   // 上
        RC_MID = 1500,  // 中
        RC_DOWN = 1000, // 下
    };

/// @brief 索引
#define RC_LX 0
#define RC_LY 1
#define RC_RX 2
#define RC_RY 3

#define RC_A 0
#define RC_B 1
#define RC_C 2
#define RC_D 3

/// @brief 摇杆
#define FSI6_JOY_LX(fsi6) (fsi6->joy[RC_LX]) // 左摇杆 横轴
#define FSI6_JOY_LY(fsi6) (fsi6->joy[RC_LY]) // 左摇杆 纵轴
#define FSI6_JOY_RX(fsi6) (fsi6->joy[RC_RX]) // 右摇杆 横轴
#define FSI6_JOY_RY(fsi6) (fsi6->joy[RC_RY]) // 右摇杆 纵轴
/// @brief 波轮
#define FSI6_VR_A(fsi6) (fsi6->vr[RC_A])
#define FSI6_VR_B(fsi6) (fsi6->vr[RC_B])
/// @brief 开关
#define FSI6_SW_A(fsi6) (fsi6->sw[RC_A])
#define FSI6_SW_B(fsi6) (fsi6->sw[RC_B])
#define FSI6_SW_C(fsi6) (fsi6->sw[RC_C])
#define FSI6_SW_D(fsi6) (fsi6->sw[RC_D])

    FSi6(void) = default;
    void Parse(uint8_t* data, uint8_t len);

    inline void Monitor(void) {}
    inline const bool IsOnline(void) {
        return is_online;
    }

private:
    static uint8_t Checksum8(const uint8_t* data, uint8_t len);

    bool is_online;
    uint32_t timeconst;

    int16_t channel[18];

    int16_t joy[4];
    int16_t vr[2];
    int16_t sw[4];
};

#endif

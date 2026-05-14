/**
 * @file power-control.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-09
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

/// @brief 电机三参数模型
/// @note
/// P = T·omega + k_1·|omega| + k_2·T^2 + k_3
/// P为电机功率，T为转矩，omega为角速度，k_1、k_2、k_3为三个参数

#ifndef POWER_CONTROL_HPP
#define POWER_CONTROL_HPP

#include "vgd-pid.hpp"

namespace vgd {
namespace module {
namespace power_control {

/// @brief 轮腿功率控制
class Wheel_Leg {
    controller::PID epid;

    float k1, k2, k3; // 电机参数

    float k,        // k = speed / yaw
        kspd, kyaw; // attenuation ratio, $k \in [0, 1]$

    float pmax, // max power setpoint
        p0;     // min(p_max) > p_0

    float A, B, C, delta, x1, x2, x;

    float eon, eoff;

public:
    Wheel_Leg(
        float k1, // 电机模型参数
        float k2,
        float k3,
        float eon,  // 超电参数：开模式期望能量
        float eoff, // 超电参数：关模式期望能量
        float kp,   // PID参数
        float kd,
        float dt
    );

    void Update(
        float* uspd,    // 速度控制量
        float* uyaw,    // yaw 控制量
        bool spcEnable, // 超电模式
        float tl,       // 左 实际扭矩
        float tr,       // 右 实际扭矩
        float omgl,     // 左 角速度
        float omgr,     // 右 角速度
        float ml,       // 左 平衡扭矩
        float mr,       // 右 平衡扭矩
        float pr,       // 当前的裁判系统最大允许功率
        float etarget,  // 目标超电能量
        float espc      // 当前超电能量
    );
};

} // namespace power_control
} // namespace module
} // namespace vgd

#endif

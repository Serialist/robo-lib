/**
 * @file power-control.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-09
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "power-control.hpp"
#include "math-utils.hpp"
#include <cmath>

namespace vgd {
namespace module {
namespace power_control {

Wheel_Leg::Wheel_Leg(
    float k1, // 电机模型参数
    float k2,
    float k3,
    float eon,  // 超电参数：开模式期望能量
    float eoff, // 超电参数：关模式期望能量
    float kp,   // PID参数
    float kd,
    float dt,
    float p0
):
    k1(k1),
    k2(k2),
    k3(k3),
    eon(eon),
    eoff(eoff),
    p0(p0) {
    epid = controller::PID(kp, 0, kd, 0, 0, dt);

    k = 0, k_limit = 0;
    pmax = 0;
    A = 0, B = 0, C = 0, delta = 0, x1 = 0, x2 = 0, x = 0;
}

/**
 * @brief 功率控制更新
 * 
 * @param uspd 速度控制量
 * @param uyaw yaw 控制量
 * @param spcEnable 超电模式使能
 * @param omgl 左 角速度
 * @param omgr 右 角速度
 * @param ml 左 平衡扭矩
 * @param mr 右 平衡扭矩
 * @param pr 当前的裁判系统最大允许功率
 * @param espc 当前超电能量
*/
void Wheel_Leg::Update(
    float* uspd,
    float* uyaw,
    bool spcEnable,
    float omgl,
    float omgr,
    float ml,
    float mr,
    float pr,
    float espc
) {
    // 能量环
    etarget = spcEnable ? eon : eoff;
    pmax = std::fmaxf(p0, pr + epid.Update(etarget, espc));
    pmax = 35;

    bool is_infs = std::fabsf(*uyaw) < 1e-3f; // 分母不能无穷小

    if (is_infs) // 所以让 uyaw = ufake = 1
        k = (*uspd);
    else
        k = (*uspd) / (*uyaw);

    // 功率环
    A = k2 * (2 * k * k + 2);
    B = 2 * k2 * (k + 1) * ml   //
        + 2 * k2 * (k - 1) * mr //
        + omgl * (k + 1)        //
        + omgr * (k - 1);
    C = omgl * ml + omgr * mr                    //
        + k1 * (std::abs(omgl) + std::abs(omgr)) //
        + k2 * (mr * mr + ml * ml)               //
        + 2 * k3 - pmax;

    delta = B * B - 4 * A * C;

    // 功率取解
    if (delta > 0) {
        float sqrt_delta = vgd::math::Sqrt(delta);

        x1 = (-B + sqrt_delta) / (2 * A);
        x2 = (-B - sqrt_delta) / (2 * A);

        bool x1_same_sign = vgd::math::Signf(x1) == vgd::math::Signf(*uyaw),
             x2_same_sign = vgd::math::Signf(x2) == vgd::math::Signf(*uyaw);

        if (x1_same_sign && x2_same_sign) {
            // 都同号选绝对值大
            x = std::fabsf(x1) > std::fabsf(x2) ? x1 : x2;
        } else if (x1_same_sign) {
            // 只一个就选这个
            x = x1;
        } else if (x2_same_sign) {
            x = x2;
        } else {
            // 否则 0
            x = 0;
        }

    } else {
        x1 = -B / (2 * A);
        if (vgd::math::Signf(x1) == vgd::math::Signf(*uyaw)) {
            x = x1;
        } else {
            x = 0;
        }
    }

    // 得到功率，并限幅

    if (is_infs) {
        k_limit = vgd::math::Clampf(x, 0, 1); // = x/ufake = x/1 = x
        (*uspd) *= k_limit;
        // uyaw 不变
    } else {
        k_limit = vgd::math::Clampf(x / (*uyaw), 0, 1);
        (*uyaw) *= k_limit;
        (*uspd) *= k_limit;
    }
}

} // namespace power_control
} // namespace module
} // namespace vgd

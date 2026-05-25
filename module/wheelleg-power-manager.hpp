/**
 * @file wheelleg-power-manager.hpp
 * @author hkustenterprize@github
 * @brief 港科轮腿功率控制开源
 * 
*/

#pragma once

#include <cstdint>

namespace rb2 {
namespace module {
namespace wl_pwctrl { // 轮腿功率控制

enum class MODE : uint8_t {
    BATTERY = 0, // 电池模式
    CAP          // 超电模式
};

void update(
    float chassisTargetSpeed,
    float chassisCurrentSpeed,
    float Uspeed,
    float Uyaw,
    float wLeftWheel,
    float leftUelse,
    float wRightWheel,
    float rightUelse
);

/// @brief 设置模式
/// @param currentMode BATTERY or CAP
void setMode(MODE currentMode);

/// @brief 获取uspeed的衰减比例
float getDecayUspeed();

/// @brief 获取uyaw的衰减比例
float getDecayUyaw();

/// @brief 获取目前目标功率上限
float getConfiguredMaxPower();

} // namespace wl_pwctrl
} // namespace module
} // namespace rb2

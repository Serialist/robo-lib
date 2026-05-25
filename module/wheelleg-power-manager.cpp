/**
 * @file wheelleg-power-manager.cpp
 * @author hkustenterprize@github
 * @brief 港科轮腿功率控制开源
 * 
*/

#include "wheelleg-power-manager.hpp"
#include "Referee_System.h"
#include "cmsis_os.h"
#include "math-utils.hpp"
#include "pid.hpp"
#include "superpower.h"

extern SuperPower_Fdb_t sp_fdb;

#define SYSTEM_TIME_INTERVAL 1
#define ABS(x) ((x) < 0 ? -(x) : (x))
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

namespace rb2 {
namespace module {
namespace wl_pwctrl {

enum class ErrorFlags {
    MotorDisconnect = 1U,   // 电机断联
    RefereeDisConnect = 2U, // 裁判系统断联
    CAPDisConnect = 4U      // 超电断联
};

float refereeMaxPower;    // 裁判系统最大上限功率
float configuredMaxPower; // 能量环最大允许功率

float powerBuff; // 当前电容剩余能量
float buffSet;   // 期望剩余能量

float chassisPower; // 当前底盘功率
float cmdPower;     // 当前控制功率
float restrictedPower;

float wLeftWheel;  // 左轮角速度 rad/s
float wRightWheel; // 左轮角速度 rad/s

float chassisTargetSpeed;  // 目标车速
float chassisCurrentSpeed; // 当前车速

float leftUelse;  // Uleg + Upitch
float rightUelse; // Uleg + Upitch

float Uspeed;
float Uyaw;

float restrictedUspeed;
float restrictedUyaw;

float decayUspeed;
float decayUyaw;

float A;
float B;
float C;
float Delta;

const float k1 = 0.2f;
const float k2 = 2.8f;
const float k3 = 3.21f;

float K; // Uspeed = K * Uyaw

MODE mode;

const TickType_t xTimeInterval = pdMS_TO_TICKS(SYSTEM_TIME_INTERVAL); // ms

controller::PID powerPD { 50.0f, 0.0f, 0.20f, 100.0f, 0 };

inline bool floatEqual(float a, float b) {
    return fabs(a - b) < 1e-5f;
}

void setMode(MODE currentMode) {
    mode = currentMode;
}

float getDecayUspeed() {
    return decayUspeed;
}
float getDecayUyaw() {
    return decayUyaw;
}
float getConfiguredMaxPower() {
    return configuredMaxPower;
}

void update(
    float chassisTargetSpeed,
    float chassisCurrentSpeed,
    float Uspeed,
    float Uyaw,
    float wLeftWheel,
    float leftUelse,
    float wRightWheel,
    float rightUelse
) {
    wl_pwctrl::chassisTargetSpeed = chassisTargetSpeed;
    wl_pwctrl::chassisCurrentSpeed = chassisCurrentSpeed;
    wl_pwctrl::Uspeed = Uspeed;
    wl_pwctrl::Uyaw = Uyaw;
    wl_pwctrl::wLeftWheel = wLeftWheel;
    wl_pwctrl::leftUelse = leftUelse;
    wl_pwctrl::wRightWheel = wRightWheel;
    wl_pwctrl::rightUelse = rightUelse;

    // u yaw speed ratio,
    if (floatEqual(Uyaw, 0.0f)) {
        if (Uspeed * Uyaw > 0.0f)
            K = 10000.0f;
        else
            K = -10000.0f;
    } else
        K = Uspeed / Uyaw;

    float leftTotalTorque = Uspeed - Uyaw + leftUelse;
    float rightTotalTorque = Uspeed + Uyaw + rightUelse;

    cmdPower = (k1 * (ABS(wLeftWheel) + ABS(wRightWheel)))
        + (k2 * (powf(leftTotalTorque, 2) + powf(rightTotalTorque, 2))) + k3
        + (wLeftWheel * leftTotalTorque) + (wRightWheel * rightTotalTorque);

    if (cmdPower <= configuredMaxPower) {
        restrictedUspeed = Uspeed;
        restrictedUyaw = Uyaw;

        decayUspeed = 1.0f;
        decayUyaw = 1.0f;
    } else {
        float speedError = chassisTargetSpeed - chassisCurrentSpeed;

        if (ABS(speedError) > 0.5f && ABS(chassisCurrentSpeed) > 0.5f
            && ((chassisTargetSpeed * chassisCurrentSpeed < 0.0f)
                || (chassisTargetSpeed >= 0.0f && speedError < 0.0f)
                || (chassisTargetSpeed <= 0.0f && speedError > 0.0f)))
        {
            decayUspeed = decayUspeed * 0.97f + 1.0f * 0.03f;
            decayUyaw = decayUyaw * 0.97f + 1.0f * 0.03f;
        } else {
            // calculate the restricted Uspeed and Uyaw
            A = (k2 * (2.0f * powf(K, 2) + 2.0f));
            B = (2.0f * k2 * (K - 1.0f) * leftUelse) + (2.0f * k2 * (K + 1.0f) * rightUelse)
                + (wLeftWheel * (K - 1.0f)) + (wRightWheel * (K + 1.0f));
            C = (wLeftWheel * leftUelse) + (wRightWheel * rightUelse)
                + (k1 * (ABS(wLeftWheel) + ABS(wRightWheel)))
                + (k2 * (powf(leftUelse, 2) + powf(rightUelse, 2))) + k3 - configuredMaxPower;

            Delta = powf(B, 2) - 4 * A * C;

            if (floatEqual(Delta, 0.0f)) // repeat roots
            {
                float tempUyaw = (-B) / (2.0f * A);

                if (tempUyaw * Uyaw > 0.0f)
                    restrictedUyaw = tempUyaw;
                else
                    restrictedUyaw = 0.0f;

                restrictedUspeed = K * restrictedUyaw;
            } else if (Delta > 0.0f) // distinct roots
            {
                float tempUyaw1 = (-B - sqrtf(Delta)) / (2.0f * A);
                float tempUyaw2 = (-B + sqrtf(Delta)) / (2.0f * A);

                if (tempUyaw1 * Uyaw > 0.0f && tempUyaw2 * Uyaw > 0.0f) {
                    if (Uyaw > 0.0f)
                        restrictedUyaw = fmax(tempUyaw1, tempUyaw2);
                    else
                        restrictedUyaw = fmin(tempUyaw1, tempUyaw2);
                } else if (tempUyaw1 * Uyaw > 0.0f)
                    restrictedUyaw = tempUyaw1;
                else if (tempUyaw2 * Uyaw > 0.0f)
                    restrictedUyaw = tempUyaw2;
                else
                    restrictedUyaw = 0.0f;

                restrictedUspeed = K * restrictedUyaw;
            } else if (Delta < 0.0f) // imaginary roots
            {
                float tempUyaw = (-B) / (2.0f * A);

                if (tempUyaw * Uyaw > 0.0f)
                    restrictedUyaw = tempUyaw;
                else
                    restrictedUyaw = 0.0f;

                restrictedUspeed = K * restrictedUyaw;
            }

            float tempDecayUspeed = restrictedUspeed / Uspeed;
            float tempDecayUyaw = restrictedUyaw / Uyaw;

            tempDecayUspeed = CLAMP(tempDecayUspeed, 0.01f, 1.00f);
            tempDecayUyaw = CLAMP(tempDecayUyaw, 0.01f, 1.00f);

            decayUspeed = tempDecayUspeed * 0.1f + decayUspeed * 0.9f;
            decayUyaw = tempDecayUyaw * 0.1f + decayUyaw * 0.9f;
        }
    }

    float restrictedLeftTotalTorque = restrictedUspeed - restrictedUyaw + leftUelse;
    float restrictedRightTotalTorque = restrictedUspeed + restrictedUyaw + rightUelse;

    restrictedPower = (k1 * (ABS(wLeftWheel) + ABS(wRightWheel)))
        + (k2 * (powf(restrictedLeftTotalTorque, 2) + powf(restrictedRightTotalTorque, 2))) + k3
        + (wLeftWheel * restrictedLeftTotalTorque) + (wRightWheel * restrictedRightTotalTorque);
}

extern "C" void powerControllTask(void* pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        /* update the current power status */
        chassisPower = sp_fdb.chassis;
        powerBuff = sp_fdb.cap;
        refereeMaxPower = Referee_System_Info.robot_status.chassis_power_limit;

        // cap power loop
        if (sp_fdb.isOnline) {
            if (mode == MODE::BATTERY) {
                buffSet = float(255 * 0.8f);
            } else {
                buffSet = float(255 * 0.4f);
            }

            configuredMaxPower = refereeMaxPower - powerPD.Update(sqrtf(buffSet), sqrtf(powerBuff));
            configuredMaxPower = fmax(configuredMaxPower, 23.0f);
        } else
            configuredMaxPower = refereeMaxPower;

        // configuredMaxPower = 60.0f;

        vTaskDelayUntil(&xLastWakeTime, xTimeInterval);
    }
}

} // namespace wl_pwctrl
} // namespace module
} // namespace rb2

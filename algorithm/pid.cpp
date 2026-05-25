/**
 * @file pid.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-10
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "pid.hpp"
#include <cstring>

namespace rb2 {
namespace controller {

PID::PID() {
    std::memset(this, 0, sizeof(PID));
}

/// @brief
/// @param kp
/// @param ki
/// @param kd
/// @param maxout 最大总输出
/// @param maxintegral 最大积分（不是最大积分输出）
PID::PID(float kp, float ki, float kd, float maxout, float maxintegral) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->maxout = maxout;
    this->maxintegral = maxintegral;

    error = 0;
    prev_error = 0;
    pprev_error = 0;
    integral = 0;
    derivative = 0;
    setpoint = 0;
    feedback = 0;
    output = 0;
}

PID::PID(float kp, float ki, float kd, float maxout, float maxintegral, float dt) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->maxout = maxout;
    this->maxintegral = maxintegral;
    this->dt = dt;

    error = 0;
    prev_error = 0;
    pprev_error = 0;
    integral = 0;
    derivative = 0;
    setpoint = 0;
    feedback = 0;
    output = 0;
}

/// @brief 设置参数
/// @param 同构造函数
void PID::ParamSet(
    float kp,
    float ki,
    float kd,
    float maxout, //
    float maxintegral
) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->maxout = maxout;
    this->maxintegral = maxintegral;
}

void PID::Reset() {
    integral = 0.0f;
    prev_error = 0.0f;
    output = 0.0f;
}

float PID::Update(float setpoint, float feedback) {
    this->setpoint = setpoint;
    this->feedback = feedback;

    // 误差
    error = setpoint - feedback;

    // 积分计算和限幅
    integral = ClampAbsf(integral + error, maxintegral);

    // 微分计算
    derivative = error - prev_error;

    // 计算
    output = kp * error    //
        + ki * integral    //
        + kd * derivative; //

    // 输出限幅
    output = ClampAbsf(output, maxout);

    // 更新
    prev_error = error;

    return output;
}

/**
 * @brief 自己赋值的pid，有时候比如de可以是陀螺仪角速度e是角度，这时候可以更准确
 * 
 * @param e 
 * @param ie 
 * @param de 
 * @return float 
*/
float PID::UpdateEZ(float e, float ie, float de) {
    error = e;
    integral = ie;
    derivative = de;

    output = kp * e + ki * ie + kd * de;
    output = ClampAbsf(output, maxout);
    return output;
}

float PID::Update_Diff(float setpoint, float feedback) {
    this->setpoint = setpoint;
    this->feedback = feedback;

    // 误差
    error = setpoint - feedback;

    // 计算
    doutput = kp * (error - prev_error)                // 比例
        + ki * error                                   // 积分
        + kd * (error - 2 * prev_error + pprev_error); // 微分

    // 输出限幅
    output += ClampAbsf(doutput, maxout);

    // 更新
    pprev_error = prev_error;
    prev_error = error;

    return output;
}

float PID::ClampAbsf(float x, float max) {
    if (x > max)
        return max;
    else if (x < -max)
        return -max;
    else
        return x;
}

} // namespace controller
} // namespace rb2

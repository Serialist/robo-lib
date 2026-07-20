/**
 * @file signal.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 信号发生器（嵌入式专用，无异常、无动态内存）
 * @version 0.1.0
 * @date 2026-07-17
 *
 * @copyright Copyright (c) Serialist 2026
 *
*/

#ifndef SIGNAL_HPP
#define SIGNAL_HPP

#include <cmath>
#include <cstdint>

namespace algorithm {
namespace signal {

/// @brief 编译期 π 常量
template<typename T>
inline constexpr T PI() {
    return T(3.14159265358979323846264338327950288419716939937510);
}

/**
 * @brief 正弦波 y = A·sin(2π·f·t + φ) + B
 *
 * @tparam T  数值类型
 * @param time        当前时间 t
 * @param amplitude   振幅 A
 * @param frequency   频率 f (Hz)
 * @param phase       初相 φ (rad)，默认 0
 * @param offset      直流偏置 B，默认 0
 * @return T          信号值
 */
template<typename T>
constexpr inline T Sin(T time, T amplitude, T frequency, T phase = T(0), T offset = T(0)) {
    return amplitude * std::sin(T(2) * PI<T> * frequency * time + phase) + offset;
}

/**
 * @brief 余弦波 y = A·cos(2π·f·t + φ) + B
 */
template<typename T>
inline T Cos(T time, T amplitude, T frequency, T phase = T(0), T offset = T(0)) {
    return amplitude * std::cos(T(2) * PI<T> * frequency * time + phase) + offset;
}

/**
 * @brief 方波
 *
 * @tparam T  数值类型
 * @param duty  占空比 [0, 1]，默认 0.5（对称方波）
 */
template<typename T>
inline T
Square(T time, T amplitude, T frequency, T duty = T(0.5), T phase = T(0), T offset = T(0)) {
    T period = T(1) / frequency;
    T t = std::fmod(time + phase / (T(2) * PI<T> * frequency), period);
    if (t < T(0)) {
        t += period;
    }
    return (t < duty * period ? amplitude : -amplitude) + offset;
}

/**
 * @brief 锯齿波（上升沿）
 */
template<typename T>
inline T Sawtooth(T time, T amplitude, T frequency, T phase = T(0), T offset = T(0)) {
    T period = T(1) / frequency;
    T t = std::fmod(time + phase / (T(2) * PI<T> * frequency), period);
    if (t < T(0)) {
        t += period;
    }
    return amplitude * (T(2) * t / period - T(1)) + offset;
}

/**
 * @brief 三角波
 */
template<typename T>
inline T Triangle(T time, T amplitude, T frequency, T phase = T(0), T offset = T(0)) {
    T period = T(1) / frequency;
    T t = std::fmod(time + phase / (T(2) * PI<T> * frequency), period);
    if (t < T(0)) {
        t += period;
    }
    T half = period / T(2);
    if (t < half) {
        return amplitude * (T(2) * t / half - T(1)) + offset;
    } else {
        return amplitude * (T(3) - T(2) * t / half) + offset;
    }
}

// ============================================================================
// 调制与复合
// ============================================================================

/**
 * @brief 线性调频（Chirp）信号
 *
 * 频率随时间线性变化: f(t) = f0 + k·t
 *
 * @param f0  起始频率 (Hz)
 * @param k   调频率 (Hz/s)
 */
template<typename T>
inline T Chirp(T time, T amplitude, T f0, T k, T phase = T(0), T offset = T(0)) {
    return amplitude * std::sin(T(2) * PI<T> * (f0 * time + T(0.5) * k * time * time) + phase)
        + offset;
}

/**
 * @brief 指数衰减正弦波 y = A·e^(-α·t)·sin(2π·f·t + φ) + B
 *
 * @param alpha  衰减系数 α，越大衰减越快
 */
template<typename T>
inline T DampedSin(T time, T amplitude, T frequency, T alpha, T phase = T(0), T offset = T(0)) {
    return amplitude * std::exp(-alpha * time) * std::sin(T(2) * PI<T> * frequency * time + phase)
        + offset;
}

/**
 * @brief 带死区的正弦波
 *
 * 在 [0, deadzone] 区间输出 0，然后按正弦上升至 amplitude
 *
 * @param deadzone  死区阈值 (0～1)，输入幅值低于此比例时输出 0
 */
template<typename T>
inline T
SinWithDeadzone(T time, T amplitude, T frequency, T deadzone, T phase = T(0), T offset = T(0)) {
    T raw = Sin(time, amplitude, frequency, phase, T(0));
    if (raw >= T(0)) {
        T thr = amplitude * deadzone;
        if (raw < thr) {
            return offset;
        }
        return (raw - thr) / (T(1) - deadzone) + offset;
    } else {
        T thr = -amplitude * deadzone;
        if (raw > thr) {
            return offset;
        }
        return (raw - thr) / (T(1) - deadzone) + offset;
    }
}

// ============================================================================
// 编译期参数化信号（零开销抽象，振幅/频率/相位编译期已知时使用）
// ============================================================================

/**
 * @brief 编译期参数化正弦波
 *
 * @tparam T          数值类型
 * @tparam AMPLITUDE  振幅（编译期常量）
 * @tparam FREQUENCY  频率 (Hz)（编译期常量）
 * @tparam PHASE      初相 (rad)（编译期常量，默认 0）
 * @tparam OFFSET     直流偏置（编译期常量，默认 0）
 * @param time        当前时间
 * @return T          信号值
 */
template<typename T, T AMPLITUDE, T FREQUENCY, T PHASE = T(0), T OFFSET = T(0)>
inline T Sin_C(T time) {
    return AMPLITUDE * std::sin(T(2) * PI<T> * FREQUENCY * time + PHASE) + OFFSET;
}

template<typename T, T AMPLITUDE, T FREQUENCY, T PHASE = T(0), T OFFSET = T(0)>
inline T Cos_C(T time) {
    return AMPLITUDE * std::cos(T(2) * PI<T> * FREQUENCY * time + PHASE) + OFFSET;
}

template<typename T, T AMPLITUDE, T FREQUENCY, T DUTY = T(0.5), T PHASE = T(0), T OFFSET = T(0)>
inline T Square_C(T time) {
    T period = T(1) / FREQUENCY;
    T t = std::fmod(time + PHASE / (T(2) * PI<T> * FREQUENCY), period);
    if (t < T(0)) {
        t += period;
    }
    return (t < DUTY * period ? AMPLITUDE : -AMPLITUDE) + OFFSET;
}

template<typename T, T AMPLITUDE, T FREQUENCY, T PHASE = T(0), T OFFSET = T(0)>
inline T Sawtooth_C(T time) {
    T period = T(1) / FREQUENCY;
    T t = std::fmod(time + PHASE / (T(2) * PI<T> * FREQUENCY), period);
    if (t < T(0)) {
        t += period;
    }
    return AMPLITUDE * (T(2) * t / period - T(1)) + OFFSET;
}

template<typename T, T AMPLITUDE, T FREQUENCY, T PHASE = T(0), T OFFSET = T(0)>
inline T Triangle_C(T time) {
    T period = T(1) / FREQUENCY;
    T t = std::fmod(time + PHASE / (T(2) * PI<T> * FREQUENCY), period);
    if (t < T(0)) {
        t += period;
    }
    T half = period / T(2);
    if (t < half) {
        return AMPLITUDE * (T(2) * t / half - T(1)) + OFFSET;
    } else {
        return AMPLITUDE * (T(3) - T(2) * t / half) + OFFSET;
    }
}

} // namespace signal
} // namespace algorithm

#endif

/**
 * @file eso.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 扩张状态观测器（Extended State Observer）
 *
 * ESO 是自抗扰控制（ADRC）的核心组件，将系统总扰动
 * （内部未建模动态 + 外部扰动）扩张为额外状态进行实时估计。
 *
 * 状态约定：
 *   x[0..N-2] — 系统各阶状态
 *   x[N-1]    — 扩张状态（总扰动 f）
 *
 * 离散预测-校正形式（current observer）：
 *   x̂⁻ = A·x̂ + B·u               (1) 预测
 *   x̂⁺ = x̂⁻ + L·(y - C·x̂⁻)       (2) 校正
 *
 * @version 0.1.0
 * @date 2026-07-20
 *
 * @copyright Copyright (c) Serialist 2026
 *
*/

#ifndef ESO_HPP
#define ESO_HPP

#include "algorithm/linear-algebra.hpp"
#include <cstddef>

namespace observer {

using algebra::Matrix;
using algebra::Vector;
using std::size_t;

template<typename T, size_t STATE_SIZE, size_t INPUT_SIZE = 1, size_t OUTPUT_SIZE = 1>
class ESO {
    static_assert(
        STATE_SIZE >= 2,
        "[observer::ESO]: STATE_SIZE must be at least 2 (1 system state + 1 extended state)."
    );

private:
    // 状态向量
    Vector<T, STATE_SIZE> x_prior;     // 先验状态（预测值）
    Vector<T, STATE_SIZE> x_posterior; // 后验状态（校正值）

    // 量测与控制输入
    Vector<T, OUTPUT_SIZE> y; // 测量值
    Vector<T, INPUT_SIZE> u;  // 控制输入

    // 系统矩阵
    Matrix<T, STATE_SIZE, STATE_SIZE> A;  // 状态转移矩阵
    Matrix<T, STATE_SIZE, INPUT_SIZE> B;  // 控制输入矩阵
    Matrix<T, OUTPUT_SIZE, STATE_SIZE> C; // 量测映射矩阵

    // 观测器增益（固定增益，由带宽 ω₀ 或极点配置离线设计）
    Matrix<T, STATE_SIZE, OUTPUT_SIZE> L; // 状态观测器增益矩阵

public:
    /**
     * @brief 构造 ESO
     *
     * @param A_   状态转移矩阵  (STATE_SIZE × STATE_SIZE)
     * @param B_   控制输入矩阵  (STATE_SIZE × INPUT_SIZE)
     * @param C_   量测映射矩阵  (OUTPUT_SIZE × STATE_SIZE)
     * @param L_   观测器增益矩阵 (STATE_SIZE × OUTPUT_SIZE)
     * @param x0   初始状态估计
     */
    ESO(const Matrix<T, STATE_SIZE, STATE_SIZE>& A_,
        const Matrix<T, STATE_SIZE, INPUT_SIZE>& B_,
        const Matrix<T, OUTPUT_SIZE, STATE_SIZE>& C_,
        const Matrix<T, STATE_SIZE, OUTPUT_SIZE>& L_,
        const Vector<T, STATE_SIZE>& x0):
        x_posterior(x0),
        A(A_),
        B(B_),
        C(C_),
        L(L_) {}

    /**
     * @brief ESO 预测-校正更新
     *
     * 1. 预测：x̂⁻ = A·x̂ + B·u
     * 2. 校正：x̂⁺ = x̂⁻ + L·(y - C·x̂⁻)
     *
     * @param y_meas  当前量测值 （OUTPUT_SIZE 维）
     * @param u_ctrl  当前控制输入（INPUT_SIZE 维）
     * @return        后验状态估计的常引用
     */
    const Vector<T, STATE_SIZE>&
    Update(const Vector<T, OUTPUT_SIZE>& y_meas, const Vector<T, INPUT_SIZE>& u_ctrl) {
        y = y_meas;
        u = u_ctrl;

        // ---------------- 预测阶段 ----------------

        // 先验状态：x̂⁻(k) = A·x̂(k-1) + B·u(k)
        x_prior = A * x_posterior + B * u;

        // ---------------- 校正阶段 ----------------

        // 后验状态：x̂(k) = x̂⁻(k) + L·(y(k) - C·x̂⁻(k))
        /// @note 使用当前量测对先验估计进行修正
        x_posterior = x_prior + L * (y - C * x_prior);

        return x_posterior;
    }

    // -------------------------------- 状态访问 --------------------------------

    /// @brief 获取后验状态估计 x̂(k)
    const Vector<T, STATE_SIZE>& State() const {
        return x_posterior;
    }

    /// @brief 获取先验状态估计 x̂⁻(k)（校正前的预测值）
    const Vector<T, STATE_SIZE>& Prior() const {
        return x_prior;
    }

    /// @brief 获取扩张状态（总扰动估计）
    /// @note 扩张状态即状态向量的最后一个分量 x[N-1]
    T Disturbance() const {
        return x_posterior[STATE_SIZE - 1];
    }
};

} // namespace observer

#endif

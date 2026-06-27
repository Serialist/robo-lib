/**
 * @file interpolation.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 拉格朗日插值(嵌入式专用，无异常、无动态内存)
 * @version 0.1.0
 * @date 2026-06-16
 *
 * @copyright Copyright (c) Serialist 2026
*/

#ifndef INTERPOLATION_HPP
#define INTERPOLATION_HPP

#include <cstdint>

namespace algorithm {

template<typename T, std::uint8_t N>
class Lagrange_Interpolation {
    static_assert(N >= 2U, "Lagrange interpolation N must be greater than or equal to 2");

public:
    Lagrange() = default;

    void construct(const T* x, const T* y) {
        for (std::uint8_t i = 0U; i < N; ++i) {
            x_buf[i] = x[i];
            y_buf[i] = y[i];
        }
    }

    // 计算基函数
    T operator()(T xt) const {
        T result = static_cast<T>(0);
        for (std::uint8_t i = 0U; i < N; ++i) {
            T basis = static_cast<T>(1);
            for (std::uint8_t j = 0U; j < N; ++j) {
                if (i != j) {
                    basis *= (xt - x_buf[j]) / (x_buf[i] - x_buf[j]);
                }
            }
            result += y_buf[i] * basis;
        }
        return result;
    }

    inline const T* get_x(void) const {
        return x_buf;
    }
    inline const T* get_y(void) const {
        return y_buf;
    }

private:
    T x_buf[N] {};
    T y_buf[N] {};
};

} // namespace algorithm

#endif

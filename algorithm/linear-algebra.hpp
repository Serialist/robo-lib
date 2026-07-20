/**
 * @file linear-algebra.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 有限维线性空间代数运算库
 * @version 0.2.0
 * @date 2026-06-27
 *
 * @copyright Copyright (c) Serialist 2026
 *
*/

#ifndef LINEAR_ALGEBRA_HPP
#define LINEAR_ALGEBRA_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace algebra {

using std::array;
using std::size_t;

// 占位用
struct Empty {};

// ================================================================
// Vector
// ================================================================

/**
 * @brief N dimension vector
 * @tparam T            element type
 * @tparam N            dimension
 * @tparam Instance     reserved slot for math optimization library instance
 */
template<typename T, size_t N, typename OPT_INSTANCE = Empty>
class Vector {
private:
    array<T, N> data;
    OPT_INSTANCE opt_instance;

    static constexpr T kEpsilon = T(1e-6); // 浮点比较容差

public:
    // -------------------------------- init --------------------------------

    constexpr Vector(): data() {};                 // 默认构造（零初始化）
    constexpr Vector(const Vector&) = default;     // 拷贝构造
    constexpr Vector(Vector&&) noexcept = default; // 移动构造

    constexpr Vector(array<T, N>& x): data(x) {} // 从数组构造
    constexpr Vector(const array<T, N>& x): data(x) {}
    template<typename... U>
    constexpr Vector(U... args): data { T(args)... } {
        static_assert(sizeof...(args) == N, "Vector: wrong arg count");
    }

    ~Vector() = default;

    Vector& operator=(const Vector&) = default;     // 拷贝赋值
    Vector& operator=(Vector&&) noexcept = default; // 移动赋值

    // -------------------------------- meta --------------------------------

    static constexpr size_t size() {
        return N;
    }

    // -------------------------------- access --------------------------------

    T& operator[](size_t i) {
        return data[i];
    }
    const T& operator[](size_t i) const {
        return data[i];
    }

    // C++11: v.template get<1>()
    // C++17: v.get<1>()
    template<size_t I>
    T& get() noexcept {
        static_assert(I < N, "[algebra::Vector::get]: index out of range");
        return data[I];
    }
    template<size_t I>
    const T& get() const noexcept {
        static_assert(I < N, "[algebra::Vector::get]: index out of range");
        return data[I];
    }

    // -------------------------------- field arithmetic --------------------------------

    // addition
    Vector& operator+=(const Vector& x) {
        for (size_t i = 0; i < N; ++i) {
            data[i] += x[i];
        }
        return *this;
    }
    Vector operator+(const Vector& x) const {
        return Vector(*this) += x;
    }

    // subtraction
    Vector& operator-=(const Vector& x) {
        for (size_t i = 0; i < N; ++i) {
            data[i] -= x[i];
        }
        return *this;
    }
    Vector operator-(const Vector& x) const {
        return Vector(*this) -= x;
    }
    Vector operator-() const {
        Vector res;
        for (size_t i = 0; i < N; ++i) {
            res[i] = -data[i];
        }
        return res;
    }

    // scalar multiplication
    Vector& operator*=(T scalar) {
        for (size_t i = 0; i < N; ++i) {
            data[i] *= scalar;
        }
        return *this;
    }
    Vector operator*(T scalar) const {
        return Vector(*this) *= scalar;
    }
    friend Vector operator*(T scalar, const Vector& x) {
        return x * scalar;
    }

    // scalar division
    Vector& operator/=(T scalar) {
        T inv = T(1) / scalar;
        for (size_t i = 0; i < N; ++i) {
            data[i] *= inv;
        }
        return *this;
    }
    Vector operator/(T scalar) const {
        return Vector(*this) /= scalar;
    }

    // comparison(with float epsilon tolerance)
    bool operator==(const Vector& x) const {
        for (size_t i = 0; i < N; ++i) {
            if (std::abs(data[i] - x[i]) > kEpsilon) {
                return false;
            }
        }
        return true;
    }
    bool operator!=(const Vector& x) const {
        return !(*this == x);
    }

    // -------------------------------- Euclidean space algebra --------------------------------

    // inner product
    T operator*(const Vector& x) const {
        T res = T(0);
        for (size_t i = 0; i < N; i++) {
            res += data[i] * x[i];
        }
        return res;
    }
    T dot(const Vector& x) const {
        return (*this) * x;
    }

    // squared of L2 norm
    T norm_squared() const {
        return dot(*this);
    }

    // L2 norm(Euclidean length)
    T norm() const {
        return std::sqrt(norm_squared());
    }

    // Euclidean distance
    static T distance(const Vector& a, const Vector& b) {
        return (a - b).norm();
    }
    T distance(const Vector& x) const {
        return distance(*this, x);
    }

    // normalize
    Vector& normalize_self() {
        T n = norm();
        if (n > kEpsilon) {
            T inv = T(1) / n;
            for (size_t i = 0; i < N; ++i) {
                data[i] *= inv;
            }
        }
        return *this;
    }
    Vector normalized() const {
        return Vector(*this).normalize_self();
    }

    // vector projection:(project a onto b)
    static Vector project(const Vector& a, const Vector& b) {
        return b * (a.dot(b) / b.norm_squared());
    }
    Vector project(const Vector& x) const {
        return project(*this, x);
    }

    // vector angle(a with b)
    static T angle(const Vector& a, const Vector& b) {
        T cos_val = a.dot(b) / (a.norm() * b.norm());
        // 防止浮点误差导致 acos 参数越界
        if (cos_val > T(1)) {
            cos_val = T(1);
        }
        if (cos_val < T(-1)) {
            cos_val = T(-1);
        }
        return std::acos(cos_val);
    }
    T angle(const Vector& x) const {
        return angle(*this, x);
    }

    // -------------------------------- utils --------------------------------

    // fill with a num
    Vector& fill(const T& val) {
        data.fill(val);
        return *this;
    }

    // return zero/one vector
    static Vector zero() {
        Vector v;
        v.fill(T(0));
        return v;
    }
    static Vector one() {
        Vector v;
        v.fill(T(1));
        return v;
    }
};

// ============================================================================
// Matrix — ROW×COL 矩阵
// ============================================================================

/**
 * @brief 矩阵模板类
 * @tparam ROW 行数
 * @tparam COL 列数
 * @tparam T   元素数值类型
 */
template<typename T, size_t ROW, size_t COL, typename OPT_INSTANCE = Empty>
class Matrix {
    static_assert(
        ROW > 0 && COL > 0,
        "[linear-algebra::matrix]: ROW and COL must be positive and nonzero."
    );

private:
    T data[ROW * COL];

    OPT_INSTANCE opt_intance;

    static constexpr T kEpsilon = T(1e-6);

public:
    // -------------------------------- init --------------------------------

    Matrix(): data {} {} // 默认构造（零初始化）
    explicit Matrix(T val) {
        fill(val);
    } // 标量填充构造

    Matrix(const Matrix&) = default;
    Matrix(Matrix&&) noexcept = default;

    constexpr Matrix(const array<T, ROW * COL>& x): data(x) {}

    template<typename... U>
    constexpr Matrix(U... args): data { T(args)... } {
        static_assert(sizeof...(args) == ROW * COL, "Matrix: wrong arg count");
    }

    Matrix& operator=(const Matrix&) = default;
    Matrix& operator=(Matrix&&) noexcept = default;

    ~Matrix() = default;

    // -------------------------------- meta --------------------------------

    static constexpr size_t row_size() {
        return ROW;
    }
    static constexpr size_t col_size() {
        return COL;
    }
    static constexpr size_t size() {
        return ROW * COL;
    }
    static constexpr bool is_square() {
        return ROW == COL;
    }

    // -------------------------------- visit --------------------------------

    T& operator()(size_t row, size_t col) {
        return data[row * col_size() + col];
    }
    const T& operator()(size_t row, size_t col) const {
        return data[row * col_size() + col];
    }

    T* data_ptr() {
        return data;
    }
    const T* data_ptr() const {
        return data;
    }

    // -------------------------------- 数域运算  --------------------------------

    // 加法
    Matrix& operator+=(const Matrix& x) {
        for (size_t i = 0; i < size(); ++i) {
            data[i] += x.data[i];
        }
        return *this;
    }
    Matrix operator+(const Matrix& x) const {
        return Matrix(*this) += x;
    }

    // 减法
    Matrix& operator-=(const Matrix& x) {
        for (size_t i = 0; i < size(); ++i) {
            data[i] -= x.data[i];
        }
        return *this;
    }
    Matrix operator-(const Matrix& x) const {
        return Matrix(*this) -= x;
    }

    Matrix operator-() const {
        Matrix res;
        for (size_t i = 0; i < size(); ++i) {
            res.data[i] = -data[i];
        }
        return res;
    }

    // 标量乘法
    Matrix& operator*=(T scalar) {
        for (size_t i = 0; i < size(); ++i) {
            data[i] *= scalar;
        }
        return *this;
    }
    Matrix operator*(T scalar) const {
        return Matrix(*this) *= scalar;
    }
    friend Matrix operator*(T scalar, const Matrix& x) {
        return x * scalar;
    }

    // 标量除法
    Matrix& operator/=(T scalar) {
        T inv = T(1) / scalar;
        for (size_t i = 0; i < size(); ++i) {
            data[i] *= inv;
        }
        return *this;
    }
    Matrix operator/(T scalar) const {
        return Matrix(*this) /= scalar;
    }

    // 相等比较（考虑浮点误差）
    bool operator==(const Matrix& x) const {
        for (size_t i = 0; i < size(); ++i) {
            if (std::abs(data[i] - x.data[i]) > kEpsilon) {
                return false;
            }
        }
        return true;
    }
    bool operator!=(const Matrix& x) const {
        return !(*this == x);
    }

    // -------------------------------- 线性变换 --------------------------------

    // 矩阵乘法  A(ROW×COL) × B(COL×COL2) → C(ROW×COL2)
    template<size_t COL2>
    Matrix<T, ROW, COL2> operator*(const Matrix<T, COL, COL2>& m) const {
        Matrix<T, ROW, COL2> res(T(0));
        for (size_t i = 0; i < ROW; ++i) {
            for (size_t k = 0; k < COL; ++k) {
                T aik = (*this)(i, k);
                for (size_t j = 0; j < COL2; ++j) {
                    res(i, j) += aik * m(k, j);
                }
            }
        }
        return res;
    }

    // 矩阵-向量乘法  A(ROW×COL) × v(COL) → u(ROW)
    Vector<T, ROW> operator*(const Vector<T, COL>& v) const {
        Vector<T, ROW> res;
        for (size_t i = 0; i < ROW; ++i) {
            T sum = T(0);
            for (size_t j = 0; j < COL; ++j) {
                sum += (*this)(i, j) * v[j];
            }
            res[i] = sum;
        }
        return res;
    }

    // 转置
    Matrix<T, COL, ROW> transpose() const {
        Matrix<T, COL, ROW> res;
        for (size_t i = 0; i < ROW; ++i) {
            for (size_t j = 0; j < COL; ++j) {
                res(j, i) = (*this)(i, j);
            }
        }
        return res;
    }

    // 秩
    // size_t rank() const {}

    // -------------------------------- 方阵专属 --------------------------------

    // 迹
    T trace() const {
        static_assert(ROW == COL, "[matrix::trace]: only square matrices have a trace.");
        T res = T(0);
        for (size_t i = 0; i < ROW; ++i) {
            res += (*this)(i, i);
        }
        return res;
    }

    // 行列式（Gauss 消元 + 部分主元选取）
    T determinant() const {
        static_assert(
            ROW == COL,
            "[matrix::determinant]: only square matrices have a determinant."
        );

        Matrix<T, ROW, COL> a(*this);
        T det = T(1);

        for (size_t col = 0; col < ROW; ++col) {
            // 部分主元选取
            size_t pivot = col;
            T max_val = std::abs(a(col, col));
            for (size_t row = col + 1; row < ROW; ++row) {
                T v = std::abs(a(row, col));
                if (v > max_val) {
                    max_val = v;
                    pivot = row;
                }
            }
            if (max_val < kEpsilon) {
                return T(0);
            } // 奇异矩阵

            // 行交换（符号翻转）
            if (pivot != col) {
                for (size_t j = 0; j < ROW; ++j) {
                    T tmp = a(col, j);
                    a(col, j) = a(pivot, j);
                    a(pivot, j) = tmp;
                }
                det = -det;
            }

            det *= a(col, col);

            // 消去下方行
            T inv_pivot = T(1) / a(col, col);
            for (size_t row = col + 1; row < ROW; ++row) {
                T factor = a(row, col) * inv_pivot;
                for (size_t j = col; j < ROW; ++j) {
                    a(row, j) -= factor * a(col, j);
                }
            }
        }
        return det;
    }

    // 逆矩阵（Gauss-Jordan 消元）
    Matrix<T, ROW, COL> inverse() const {
        static_assert(ROW == COL, "[matrix::inverse]: only square matrices are invertible.");

        // 增广矩阵 [*this | I]
        Matrix<T, ROW, COL * 2> aug(T(0));
        for (size_t i = 0; i < ROW; ++i) {
            for (size_t j = 0; j < COL; ++j) {
                aug(i, j) = (*this)(i, j);
            }
            aug(i, COL + i) = T(1);
        }

        constexpr size_t AUG_COL = COL * 2;

        // 前向消元 + 归一化
        for (size_t col = 0; col < ROW; ++col) {
            // 部分主元选取
            size_t pivot = col;
            T max_val = std::abs(aug(col, col));
            for (size_t row = col + 1; row < ROW; ++row) {
                T v = std::abs(aug(row, col));
                if (v > max_val) {
                    max_val = v;
                    pivot = row;
                }
            }
            if (max_val < kEpsilon) {
                return zero();
            } // 不可逆

            // 行交换
            if (pivot != col) {
                for (size_t j = 0; j < AUG_COL; ++j) {
                    T tmp = aug(col, j);
                    aug(col, j) = aug(pivot, j);
                    aug(pivot, j) = tmp;
                }
            }

            // 主元归一化
            T inv = T(1) / aug(col, col);
            for (size_t j = 0; j < AUG_COL; ++j) {
                aug(col, j) *= inv;
            }

            // 消去其他所有行（不仅仅是下方）
            for (size_t row = 0; row < ROW; ++row) {
                if (row == col) {
                    continue;
                }
                T factor = aug(row, col);
                for (size_t j = 0; j < AUG_COL; ++j) {
                    aug(row, j) -= factor * aug(col, j);
                }
            }
        }

        // 提取逆矩阵
        Matrix<T, ROW, COL> res;
        for (size_t i = 0; i < ROW; ++i) {
            for (size_t j = 0; j < COL; ++j) {
                res(i, j) = aug(i, COL + j);
            }
        }
        return res;
    }

    // -------------------------------- 工具 --------------------------------

    Matrix& fill(T val) {
        for (size_t i = 0; i < size(); ++i) {
            data[i] = val;
        }
        return *this;
    }

    // Frobenius 范数
    T norm() const {
        T sum = T(0);
        for (size_t i = 0; i < size(); ++i) {
            sum += data[i] * data[i];
        }
        return std::sqrt(sum);
    }

    static Matrix zero() {
        return Matrix(T(0));
    }
    static Matrix one() {
        return Matrix(T(1));
    }

    static Matrix eye() {
        static_assert(ROW == COL, "[matrix::eye]: identity only defined for square matrices.");
        Matrix res(T(0));
        for (size_t i = 0; i < ROW; ++i) {
            res(i, i) = T(1);
        }
        return res;
    }
};

// ============================================================================
// 类型别名
// ============================================================================

template<size_t N>
using Vectorf = Vector<float, N>;
using Vector2f = Vector<float, 2>;
using Vector3f = Vector<float, 3>;
using Vector4f = Vector<float, 4>;

template<size_t N>
using Vectord = Vector<double, N>;
using Vector2d = Vector<double, 2>;
using Vector3d = Vector<double, 3>;
using Vector4d = Vector<double, 4>;

template<size_t ROW, size_t COL>
using Matrixf = Matrix<float, ROW, COL>;
using Matrix2f = Matrix<float, 2, 2>;
using Matrix3f = Matrix<float, 3, 3>;
using Matrix4f = Matrix<float, 4, 4>;

template<size_t ROW, size_t COL>
using Matrixd = Matrix<double, ROW, COL>;
using Matrix2d = Matrix<double, 2, 2>;
using Matrix3d = Matrix<double, 3, 3>;
using Matrix4d = Matrix<double, 4, 4>;

} // namespace algebra

#endif

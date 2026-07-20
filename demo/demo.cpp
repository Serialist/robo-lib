/**
 * @file demo.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief ���Դ��� & �������˲��� ������ʾ�����
 * @version 0.1.0
 * @date 2026-07-19
 *
 * @copyright Copyright (c) Serialist 2026
 *
*/

#include "algorithm/kalman-filter.hpp"
#include "algorithm/linear-algebra.hpp"
#include "algorithm/quaternion.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>

using namespace std;
using namespace algebra;
using namespace observer;

// ============================================================
// ������أ����ڵ��ԣ�
// ============================================================

template<size_t DIM>
ostream& operator<<(ostream& os, const Vector<float, DIM>& v) {
    os << "[";
    for (size_t i = 0; i < DIM; i++) {
        os << v[i];
        if (i + 1 < DIM)
            os << ", ";
    }
    os << "]";
    return os;
}

template<size_t ROW, size_t COL>
ostream& operator<<(ostream& os, const Matrix<float, ROW, COL>& m) {
    for (size_t i = 0; i < m.row_size(); i++) {
        os << (i == 0 ? "[" : " ");
        for (size_t j = 0; j < m.col_size(); j++) {
            os << m(i, j);
            if (j + 1 < m.col_size())
                os << "\t";
        }
        os << (i + 1 == m.row_size() ? "]" : "") << "\n";
    }
    return os;
}

// ============================================================
// ���� 1�����Դ�����������
// ============================================================
void test_linear_algebra() {
    cout << "========== ���Դ����������� ==========\n" << endl;

    // ����˷�
    Matrix<float, 3, 2> m;
    m(0, 0) = 2;
    m(0, 1) = 0;
    m(1, 0) = 0;
    m(1, 1) = 0;
    m(2, 0) = 3;
    m(2, 1) = 0;

    auto I3 = Matrix<float, 3, 3>::eye();

    cout << "I3 * m =" << endl;
    cout << I3 * m << endl;

    // ��������
    Vector<float, 3> a;
    a[0] = 3;
    a[1] = 0;
    a[2] = 0;
    Vector<float, 3> b;
    b[0] = 0;
    b[1] = 4;
    b[2] = 0;

    cout << "a = " << a << endl;
    cout << "b = " << b << endl;
    cout << "a + b = " << a + b << endl;
    cout << "a �� b = " << a.dot(b) << endl;
    cout << "||a|| = " << a.norm() << endl;
    cout << "distance(a, b) = " << Vector<float, 3>::distance(a, b) << endl;

    // ����ת�� & ��
    Matrix<float, 2, 2> M;
    M(0, 0) = 4;
    M(0, 1) = 7;
    M(1, 0) = 2;
    M(1, 1) = 6;

    cout << "\nM = " << endl << M << endl;
    cout << "M^T = " << endl << M.transpose() << endl;
    cout << "det(M) = " << M.determinant() << endl;
    cout << "M^{-1} = " << endl << M.inverse() << endl;
    cout << "M * M^{-1} = " << endl << M * M.inverse() << endl;
}

// ============================================================
// ���� 2���������˲��� �� 1D ��������
// ============================================================
void test_kalman_1d_constant() {
    cout << "\n========== �������˲�����1D �������� ==========\n" << endl;

    // STATE = 1������ֵ����MEASURE = 1���۲�ֵ��
    constexpr size_t STATE = 1;
    constexpr size_t MEASURE = 1;

    // ϵͳ����
    Matrix<float, STATE, STATE> F(1.0f);      // ״̬ת�ƣ�x' = x
    Matrix<float, MEASURE, STATE> H(1.0f);    // �۲�ӳ�䣺z = x
    Matrix<float, STATE, STATE> Q(0.001f);    // ��������Э���С��
    Matrix<float, MEASURE, MEASURE> R(0.25f); // �۲�����Э�����? �� 0.5?��

    Vector<float, STATE> x0;
    x0[0] = 0.0f;                         // ��ʼ���� = 0
    Matrix<float, STATE, STATE> P0(1.0f); // ��ʼЭ���� = 1

    Kalman_Filter<float, MEASURE, STATE> kf(F, H, Q, R, x0, P0);

    // ��ʵֵ = 10.0������ȷ����"����"�Ĺ۲�����
    const float true_value = 10.0f;
    const float measurements[] = {
        9.8f, 10.3f, 9.5f, 10.1f, 10.4f, 9.7f, 10.0f, 10.2f, 9.9f, 9.6f
    };
    constexpr int N = sizeof(measurements) / sizeof(measurements[0]);

    cout << "��ʵֵ: " << true_value << endl;
    cout << "�������� Q = " << Q(0, 0) << ", �۲����� R = " << R(0, 0) << "\n" << endl;
    cout << "����\t�۲�ֵ\t\t����ֵ\t\t���" << endl;
    cout << "----\t------\t\t------\t\t----" << endl;

    for (int i = 0; i < N; i++) {
        Vector<float, MEASURE> z;
        z[0] = measurements[i];
        const auto& est = kf.Update(z);
        float err = est[0] - true_value;
        cout << i << "\t" << z[0] << "\t\t" << est[0] << "\t\t" << err << endl;
    }
    cout << "\n�� ����ֵӦ��������ʵֵ 10.0 ����" << endl;
}

// ============================================================
// ���� 3���������˲��� �� 2D ����ģ�ͣ�λ��-�ٶȹ��ƣ�
// ============================================================
void test_kalman_2d_cv() {
    cout << "\n========== �������˲�����2D ����ģ�� ==========\n" << endl;

    // STATE = 2��λ��, �ٶȣ���MEASURE = 1����λ�ù۲⣩
    constexpr size_t STATE = 2;
    constexpr size_t MEASURE = 1;

    const float dt = 0.1f; // ʱ�䲽��

    // F = [[1, dt], [0, 1]]
    Matrix<float, STATE, STATE> F(0.0f);
    F(0, 0) = 1.0f;
    F(0, 1) = dt;
    F(1, 0) = 0.0f;
    F(1, 1) = 1.0f;

    // H = [[1, 0]]
    Matrix<float, MEASURE, STATE> H(0.0f);
    H(0, 0) = 1.0f;
    H(0, 1) = 0.0f;

    // �������� Q����С��
    Matrix<float, STATE, STATE> Q(0.0f);
    Q(0, 0) = 0.01f;
    Q(1, 1) = 0.01f;

    // �۲����� R
    Matrix<float, MEASURE, MEASURE> R(1.0f);

    // ��ʼ����
    Vector<float, STATE> x0;
    x0[0] = 0.0f; // ��ʼλ�ù���
    x0[1] = 0.0f; // ��ʼ�ٶȹ���

    // P0 = I
    Matrix<float, STATE, STATE> P0 = Matrix<float, STATE, STATE>::eye();

    Kalman_Filter<float, MEASURE, STATE> kf(F, H, Q, R, x0, P0);

    // ��ʵ�˶�����ʼλ�� 0���ٶ� 2.0
    const float true_vel = 2.0f;
    float true_pos = 0.0f;

    // ȷ���������������
    mt19937 gen(42);
    normal_distribution<float> noise(0.0f, 1.0f);

    cout << "��ʵ�ٶ�: " << true_vel << ", dt = " << dt << endl;
    cout << "�۲����λ�ã������� ��=1.0��\n" << endl;
    cout << "����\t��ʵλ��\t�۲�λ��\t����λ��\t�����ٶ�" << endl;
    cout << "----\t--------\t--------\t--------\t--------" << endl;

    for (int i = 0; i < 15; i++) {
        true_pos += true_vel * dt; // ��ʵλ���ƽ�

        float measured_pos = true_pos + noise(gen);

        Vector<float, MEASURE> z;
        z[0] = measured_pos;
        const auto& est = kf.Update(z);

        cout << i << "\t" << true_pos << "\t\t" << measured_pos << "\t\t" << est[0] << "\t\t"
             << est[1] << endl;
    }
    cout << "\n�� λ�ù���Ӧ��������ʵ�켣���ٶȹ���Ӧ������ 2.0 ����" << endl;
}

// ============================================================
// ���� 4����������
// ============================================================
void test_quaternion() {
    cout << "\n========== ������������ ==========\n" << endl;

    using Quat = Quaternion<float>;

    // --- ��������� ---
    Quat q0; // Ĭ�Ϲ��죺��λ��Ԫ��
    cout << "default (identity): " << q0 << endl;
    cout << "  w=" << q0.w() << " x=" << q0.x() << " y=" << q0.y() << " z=" << q0.z() << endl;

    Quat q1(0, 1, 0, 0); // �� x ����ת 180�� (���������תΪπ)
    cout << "\nq1 (180�� around x): " << q1 << endl;

    // --- Hamilton �˻� ---
    Quat qx = Quat::FromAxisAngle({1.0f, 0.0f, 0.0f}, Quat::PI() * 0.5f);
    Quat qy = Quat::FromAxisAngle({0.0f, 1.0f, 0.0f}, Quat::PI() * 0.5f);
    Quat qz = Quat::FromAxisAngle({0.0f, 0.0f, 1.0f}, Quat::PI() * 0.5f);

    cout << "\nqx (90�� around X): " << qx << endl;
    cout << "qy (90�� around Y): " << qy << endl;
    cout << "qz (90�� around Z): " << qz << endl;

    Quat qxy = qx * qy; // �Ȼ� Y �ٻ� X
    cout << "qx * qy: " << qxy << endl;

    // --- ������ʽ ---
    cout << "\n--- ������ʽ ---" << endl;
    cout << "qx + qy: " << qx + qy << endl;
    cout << "qx - qy: " << qx - qy << endl;
    cout << "qx * 2.0: " << qx * 2.0f << endl;
    cout << "3.0 * qx: " << 3.0f * qx << endl;
    cout << "qx / 2.0: " << qx / 2.0f << endl;
    cout << "-qy: " << -qy << endl;

    // --- ������Ƚ� ---
    Quat qx2 = qx;
    cout << "\nqx == qx2: " << (qx == qx2 ? "true" : "false") << endl;
    cout << "qx != qy: " << (qx != qy ? "true" : "false") << endl;

    // --- ��� + �� ---
    cout << "\n--- ��� + �� ---" << endl;
    Quat qx_conj = qx.Conjugate();
    cout << "Conjugate(qx): " << qx_conj << endl;
    Quat qx_inv = qx.Inverse();
    cout << "Inverse(qx): " << qx_inv << endl;
    cout << "qx * Inverse(qx): " << qx * qx_inv << "  (should be identity)" << endl;
    cout << "qx * Conjugate(qx) [unit]: " << qx * qx_conj << endl;

    // --- ��ת���� ---
    cout << "\n--- ������ת ---" << endl;
    array<float, 3> v = {1.0f, 0.0f, 0.0f};
    auto v_rot = qz.Rotate(v); // �� Z ��ת 90�㣬(1,0,0) -> (0,1,0)
    cout << "v = [" << v[0] << ", " << v[1] << ", " << v[2] << "]" << endl;
    cout << "qz.Rotate(v) = [" << v_rot[0] << ", " << v_rot[1] << ", " << v_rot[2] << "]  (expect ~[0,1,0])" << endl;

    // --- Euler �� <-> ��Ԫ�� ---
    cout << "\n--- Euler�� <-> ��Ԫ�� ---" << endl;
    float r = 0.1f, p = 0.2f, y = 0.3f;
    Quat q_euler = Quat::FromEulerAngles(r, p, y);
    cout << "FromEuler(" << r << ", " << p << ", " << y << "): " << q_euler << endl;
    auto euler_back = q_euler.ToEulerAngles();
    cout << "ToEuler back: [" << euler_back[0] << ", " << euler_back[1] << ", " << euler_back[2] << "]" << endl;
    cout << "  round-trip error: "
         << "roll=" << (euler_back[0] - r) << " pitch=" << (euler_back[1] - p) << " yaw=" << (euler_back[2] - y)
         << endl;

    // --- ���� <-> ��Ԫ�� ---
    cout << "\n--- ���� <-> ��Ԫ�� ---" << endl;
    auto [axis, angle] = qz.ToAxisAngle();
    cout << "qz.ToAxisAngle: axis=[" << axis[0] << ", " << axis[1] << ", " << axis[2] << "], angle=" << angle
         << " (~PI/2=" << (Quat::PI() * 0.5f) << ")" << endl;
    Quat qz_from_aa = Quat::FromAxisAngle(axis, angle);
    cout << "FromAxisAngle back: " << qz_from_aa << endl;

    // --- ��ת���� <-> ��Ԫ�� ---
    cout << "\n--- ��ת���� <-> ��Ԫ�� ---" << endl;
    auto R = qz.ToRotationMatrix();
    cout << "qz.ToRotationMatrix:" << endl;
    for (int i = 0; i < 3; i++) {
        cout << "  [" << R[i][0] << ", " << R[i][1] << ", " << R[i][2] << "]" << endl;
    }
    Quat qz_from_R = Quat::FromRotationMatrix(R);
    cout << "FromRotationMatrix back: " << qz_from_R << endl;

    // --- Slerp / Nlerp ---
    cout << "\n--- Slerp / Nlerp ---" << endl;
    Quat q_identity = Quat::Identity();
    cout << "Slerp(I, qz, 0.0): " << Quat::Slerp(q_identity, qz, 0.0f) << endl;
    cout << "Slerp(I, qz, 0.5): " << Quat::Slerp(q_identity, qz, 0.5f) << endl;
    cout << "Slerp(I, qz, 1.0): " << Quat::Slerp(q_identity, qz, 1.0f) << endl;
    cout << "Nlerp(I, qz, 0.5): " << Quat::Nlerp(q_identity, qz, 0.5f) << endl;

    // --- Log / Exp / Pow ---
    cout << "\n--- Log / Exp / Pow ---" << endl;
    auto qz_log = qz.Log();
    cout << "Log(qz): " << qz_log << "  (pure quaternion)" << endl;
    auto qz_exp_back = qz_log.Exp();
    cout << "Exp(Log(qz)): " << qz_exp_back << "  (should ~= qz)" << endl;
    auto qz_pow_half = qz.Pow(0.5f);
    cout << "qz.Pow(0.5): " << qz_pow_half << endl;
    cout << "qz.Pow(0.5) * qz.Pow(0.5): " << qz_pow_half * qz_pow_half << "  (should ~= qz)" << endl;

    // --- ���ߣ�IsUnit, Angle, Opposite ---
    cout << "\n--- ���ߺ��� ---" << endl;
    cout << "q_identity.IsUnit(): " << (q_identity.IsUnit() ? "true" : "false") << endl;
    cout << "qz.IsUnit(): " << (qz.IsUnit() ? "true" : "false") << endl;
    cout << "Angle(I, qz): " << Quat::Angle(q_identity, qz) << " rad (~PI/2)" << endl;
    cout << "Opposite(qz): " << qz.Opposite() << "  (same rotation as qz)" << endl;

    // --- dot (�̳л���) ---
    cout << "\n--- dot (base class) ---" << endl;
    cout << "qx.dot(qx): " << qx.dot(qx) << "  (unit quaternion -> 1)" << endl;
    cout << "qx.dot(qy): " << qx.dot(qy) << endl;

    cout << "\n�� ����������ͨ����" << endl;
}

// ============================================================
// ������
// ============================================================
int main() {
    test_linear_algebra();
    test_kalman_1d_constant();
    test_kalman_2d_cv();
    test_quaternion();

    cout << "\n���в�����ɡ�" << endl;
    return 0;
}

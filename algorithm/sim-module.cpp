#include "sim-module.hpp"

pt::SecondOrderSystem::SecondOrderSystem(double mass, double damping)
	: m(mass), kf(damping) {}

/**
 * @brief 获取当前位置
 *
 * @return double
 */
double pt::SecondOrderSystem::position() { return x; }

/**
 * @brief 单步更新
 *
 * @param u
 * @param dt
 */
void pt::SecondOrderSystem::step(double u, double dt)
{
	// 二阶系统动力学：加速度 a = (u - kf*v) / m
	double a = (u - kf * v) / m;

	// 欧拉积分（简单稳定）
	v = v + a * dt; // 更新速度
	x = x + v * dt; // 更新位置
}

/**
 * @brief 重置状态
 *
 */
void pt::SecondOrderSystem::reset()
{
	x = 0.0;
	v = 0.0;
}
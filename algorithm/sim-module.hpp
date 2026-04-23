/**
 * @file sim-module.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-23
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef SIM_MODULE_HPP
#define SIM_MODULE_HPP

namespace pt
{
	// 二阶惯性系统
	class SecondOrderSystem
	{
		// 状态变量
		double x = 0.0; // 位置
		double v = 0.0; // 速度

		// 系统参数
		double m;  // 质量
		double kf; // 阻尼系数

	public:
		// 构造函数
		SecondOrderSystem(double mass, double damping)
			: m(mass), kf(damping) {}

		// 获取当前位置
		double position() { return x; }

		// 单步更新（核心：正确二阶系统离散求解）
		void step(double u, double dt)
		{
			// 二阶系统动力学：加速度 a = (u - kf*v) / m
			double a = (u - kf * v) / m;

			// 欧拉积分（简单稳定）
			v = v + a * dt; // 更新速度
			x = x + v * dt; // 更新位置
		}

		// 可选：重置状态
		void reset()
		{
			x = 0.0;
			v = 0.0;
		}
	};
}

#endif

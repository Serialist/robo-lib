/**
 * @file power-control.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-09
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "power-control.hpp"
#include "math-utils.hpp"
#include <cmath>

namespace vgd {
namespace module {
namespace power_control {

Wheel_Leg::Wheel_Leg(
	float k1, // 电机模型参数
	float k2,
	float k3,
	float eon,	// 超电参数：开模式期望能量
	float eoff, // 超电参数：关模式期望能量
	float kp,	// PID参数
	float kd,
	float dt
):
	k1(k1),
	k2(k2),
	k3(k3),
	eon(eon),
	eoff(eoff) {
	epid = controller::PID(kp, 0, kd, 0, 0, dt);

	k = 0, kspd = 0, kyaw = 0;
	pmax = 0, p0 = 0;
	A = 0, B = 0, C = 0, delta = 0, x1 = 0, x2 = 0, x = 0;
}

void Wheel_Leg::Update(
	float* uspd,	// 速度控制量
	float* uyaw,	// yaw 控制量
	bool spcEnable, // 超电模式
	float tl,		// 左 实际扭矩
	float tr,		// 右 实际扭矩
	float omgl,		// 左 角速度
	float omgr,		// 右 角速度
	float ml,		// 左 平衡扭矩
	float mr,		// 右 平衡扭矩
	float pr,		// 当前的裁判系统最大允许功率
	float etarget,	// 目标超电能量
	float espc		// 当前超电能量
) {
	// 能量环
	etarget = spcEnable ? eon : eoff;
	pmax = epid.Update(etarget, espc);

	// 功率环
	k = *uspd / *uyaw;
	A = k2 * (2 * k * k + 2);
	B = 2 * k2 * (k + 1) * ml	//
		+ 2 * k2 * (k - 1) * mr //
		+ omgl * (k + 1)		//
		+ omgr * (k - 1);
	C = omgl * ml + omgr * mr					 //
		+ k1 * (std::abs(omgl) + std::abs(omgr)) //
		+ k2 * (mr * mr + ml * ml)				 //
		+ k3 - pmax;

	delta = B * B - 4 * A * C;
	// 功率取解
	if (delta > 0) {
		float sqrt_delta = vgd::math::Sqrt(delta);

		x1 = (-B + sqrt_delta) / (2 * A);
		x2 = (-B - sqrt_delta) / (2 * A);

		if (vgd::math::Sign(x1) == vgd::math::Sign(*uyaw))
			x = x1;

		if (vgd::math::Sign(x2) == vgd::math::Sign(*uyaw))
			x = vgd::math::Sign(*uyaw) ? std::fminf(x, x2) : std::fmaxf(x, x2);

	} else {
		x = -B / (2 * A);

		if (vgd::math::Sign(x) != vgd::math::Sign(*uyaw)) {
			x = 0;
		}
	}

	// 得到功率，并限幅
	kyaw = vgd::math::Clampf(x / (*uyaw), 0, 1);
	kspd = vgd::math::Clampf(k * x / (*uyaw), 0, 1);
	*uyaw = kyaw * (*uyaw);
	*uspd = kspd * (*uspd);
}

} // namespace power_control
} // namespace module
} // namespace vgd

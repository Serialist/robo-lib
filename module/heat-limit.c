
#include "heat-limit.h"

#define BULLET_HEAT 10.0f	   // 单发热量
#define GEAR_RATIO 36.0f	   // 2006减速比
#define SMELL_GEAR_RATIO 2.5f  // 大小齿轮比2：5
#define SLOTS_ON_PLATE 10.0f   // 拨盘弹丸数
#define COOLING_SAFE_GAP 21.0f // 热量留余
#define SHOOT_MOTOR_DARTLE_SPEED_SET 500

/**
 * @brief 计算弹丸的 firing rate 以满足热量限制
 *
 * @param cooling_speed 冷却速度
 * @param heat_limit 热量限制
 * @param heat_per_bullet 单发热量
 */
float Heat_Limit(float cooling_speed, float heat_limit, float heat_per_bullet)
{
	float shoot_freq = 0;
	// if (Mode.Shoot_Mode == SINGLE || Mode.Fire_Mode == FIRE_REVERSAL)
	// 	return;

	// float a_cooling = REF.GameRobotStat.shooter_barrel_cooling_value;
	float current_heat = heat_limit - heat_per_bullet;

	if (1 /* is fire */)
	{
		float m_left = current_heat - COOLING_SAFE_GAP; //
		float target_fire_rate = 0.0f;

		if (m_left > 0)
			target_fire_rate = (cooling_speed / BULLET_HEAT) + (m_left * 0.1f);
		else
			target_fire_rate = cooling_speed / BULLET_HEAT;
		float max_safe_rpm = (target_fire_rate * 60.0f / SLOTS_ON_PLATE) *
							 GEAR_RATIO * SMELL_GEAR_RATIO;

		if ((int16_t)max_safe_rpm > SHOOT_MOTOR_DARTLE_SPEED_SET)
			shoot_freq = SHOOT_MOTOR_DARTLE_SPEED_SET;
		else
			shoot_freq = (int16_t)max_safe_rpm;
	}
	else
		shoot_freq = 0;

	return shoot_freq;
}
/**
 * @file mecanum.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-13
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "mecanum.h"

void Mechanum_Init(Mecanum_Chassis_t *ch, float rw, float lx, float ly)
{
	ch->rw = rw;
	ch->lx = lx;
	ch->ly = ly;
	ch->spin = lx + ly;
}

void Mechanum_FK(Mecanum_Wheel_t *w, Mecanum_Chassis_t *ch)
{
	ch->vx = (w->v1 + w->v2 + w->v3 + w->v4) / 4;
	ch->vy = (w->v1 - w->v2 + w->v3 - w->v4) / 4;
	ch->vyaw = (w->v1 + w->v2 - w->v3 - w->v4) / (4 * ch->spin);
}

void Mechanum_IK(Mecanum_Chassis_t *ch, Mecanum_Wheel_t *w)
{
	w->v1 = -ch->vx - ch->vy - ch->vyaw;
	w->v2 = -ch->vx + ch->vy - ch->vyaw;
	w->v3 = ch->vx - ch->vy - ch->vyaw;
	w->v4 = ch->vx + ch->vy - ch->vyaw;
}

void Mechanum_IKez(Mecanum_Chassis_t *ch, Mecanum_Wheel_t *w)
{
	w->v1 = (-ch->vx - ch->vy - ch->spin * ch->vyaw) / ch->rw;
	w->v2 = (-ch->vx + ch->vy - ch->spin * ch->vyaw) / ch->rw;
	w->v3 = (ch->vx - ch->vy - ch->spin * ch->vyaw) / ch->rw;
	w->v4 = (ch->vx + ch->vy - ch->spin * ch->vyaw) / ch->rw;
}

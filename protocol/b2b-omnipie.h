/**
 * @file b2b-omnipie.h
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-07-30
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef B2B_OMNIPIE_H
#define B2B_OMNIPIE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct B2B_OmniPie_Data {
    // both used
    int8_t reset_flag;
    // frame 0
    float vx_set;
    float vy_set;
    float vyaw_set;
    // frame 1
    bool spinbot;
    bool nospinbot;
    bool zero_force;
    bool fricWheel_enable;
    bool fire_enable;
    bool aimbot_seen;
    bool aimbot_fire;
    float pitch_angle;
    int16_t shoot_current_set;
    // frame 2
    float initial_speed;
    int16_t shooter_barrel_heat;
    int16_t shooter_barrel_heat_limit;
    int16_t shooter_barrel_cooling_value;
};

// debug var for UI
extern int16_t debug_ui[4];

// id 0x21;
void B2B_OmniPie_CtrlMsg_Encode(struct B2B_OmniPie_Data* data, uint8_t* buf);
// id 0x22
void B2B_OmniPie_InfoMsg_Encode(struct B2B_OmniPie_Data* data, uint8_t* buf);
// id 0x23
void B2B_OmniPie_RefMsg_Decode(uint8_t* buf, struct B2B_OmniPie_Data* data);

#ifdef __cplusplus
}
#endif

#endif

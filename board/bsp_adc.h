#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "user_lib.h"

extern void init_vrefint_reciprocal(void);
extern float32_t get_temprate(void);
extern float32_t get_battery_voltage(void);
#endif

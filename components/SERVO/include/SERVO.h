/**
 * @file LED.h
 * @author bignut
 * @brief 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __LTL_SERVO__
#define __LTL_SERVO__

#include "driver/gpio.h"


#ifdef __cplusplus
extern "C" {
#endif



void servo_pwm_init_ltl();

void servo_set_pwm_0(double percent);

#ifdef __cplusplus
}
#endif

#endif
/**
 * @file MOTROR.h
 * @author bignut
 * @brief 
 * @details
 *      esp32-wroom32 
 *          GPIO21  MOTOR_AIN1
 *          GPIO22  MOTOR_AIN2
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __LTL_MOTOR__
#define __LTL_MOTOR__

#include "stdio.h"


#ifdef __cplusplus
extern "C" {
#endif

void motor_init();

void motor_control(int speed_mm_s, double angle_arc);


#ifdef __cplusplus
}
#endif

#endif
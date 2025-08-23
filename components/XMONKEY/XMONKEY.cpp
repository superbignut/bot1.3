/**
 * @file MONKEY.c
 * @author bignut
 * @brief
 * @version 0.1
 * @date 2025-08-5
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "XMONKEY.h"
#include "stdio.h"
#include "esp_timer.h"
#include "PCA9685.h"

void INNER_MOTOR::set_motor_angle(float angle)
{
    MY_PCA9685_SET_ANGLE(_leg_index, angle - _offset);
}
INNER_MOTOR::INNER_MOTOR()
{

}

INNER_MOTOR::INNER_MOTOR(int leg_index, int offset)
{
    _leg_index = leg_index;
    _offset = offset;
}

void OUTER_MOTOR::set_motor_angle(float angle)
{
    MY_PCA9685_SET_ANGLE(_leg_index, angle - _offset);
}

OUTER_MOTOR::OUTER_MOTOR()
{
    
}

OUTER_MOTOR::OUTER_MOTOR(int leg_index, int offset)
{
    _leg_index = leg_index;
    _offset = offset;
}

LEG::LEG()
{
    
}
LEG::LEG(int in_index,int in_offset,int ot_index,int ot_offset)
{   

    in_motor.set_leg_index(in_index);
    in_motor.set_leg_offset(in_offset);
}

MONKEY::MONKEY()
{   
    MY_PCA9685_Init();

    /*
        LEG init, 注册相对位置


    */

    printf("init...\n");    
    
}

void MONKEY::walk(float steps, int T = 500)
{
    printf("walk...\n");


}

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

}

void LEG::set_leg(int in_index,int in_offset,int ot_index,int ot_offset)
{
    in_motor.set_leg_index(in_index);
    in_motor.set_leg_offset(in_offset);

    ot_motor.set_leg_index(ot_index);
    ot_motor.set_leg_offset(ot_offset);
}


/// @brief robot 初始化 机器人固定参数
MONKEY::MONKEY()
{   
    MY_PCA9685_Init();

    /*
        LEG init, 注册编号， offset
    */
    _leg[0].set_leg(4, 0, 0, 0);  // leg 初始化 编号和offset
    _leg[1].set_leg(5, 0, 1, 0);  // leg 初始化 编号和offset
    _leg[2].set_leg(6, 0, 2, 0);  // leg 初始化 编号和offset
    _leg[3].set_leg(7, 0, 3, 0);  // leg 初始化 编号和offset


    set_leg_rela(LEG_0, 0, 0);
    set_leg_rela(LEG_1, 0, 0);  // <--------------- Todo 
    set_leg_rela(LEG_2, 0, 0);
    set_leg_rela(LEG_3, 0, 0);

    printf("init...\n");    
    
}


void MONKEY::set_leg_rela(int leg_index, float rela_x, float rela_y)
{
    assert(leg_index >=0 && leg_index <LEG_NUM);

    _leg_rela_x[leg_index] = rela_x;
    _leg_rela_y[leg_index] = rela_y;
}



void MONKEY::reset()
{

}


/// @brief monkey 所有 leg 初始化
void MONKEY::reset()
{

}

void MONKEY::walk(float steps, int T = 500)
{
    printf("walk...\n");


}

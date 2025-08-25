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

/* void INNER_MOTOR::set_motor_angle(float angle)
{
    _angle = angle;
    
}
INNER_MOTOR::INNER_MOTOR()
{

}

INNER_MOTOR::INNER_MOTOR(int leg_index, int offset)
{
    _leg_index = leg_index;
    _offset = offset;
}

void INNER_MOTOR::motor_exec()
{
    MY_PCA9685_SET_ANGLE(_leg_index, _angle - _offset);
}

void OUTER_MOTOR::set_motor_angle(float angle)
{
    _angle = angle;
    // MY_PCA9685_SET_ANGLE(_leg_index, angle - _offset);
}

OUTER_MOTOR::OUTER_MOTOR()
{
    
}

OUTER_MOTOR::OUTER_MOTOR(int leg_index, int offset)
{
    _leg_index = leg_index;
    _offset = offset;
}

void OUTER_MOTOR::motor_exec()
{
    MY_PCA9685_SET_ANGLE(_leg_index, _angle - _offset);
} */

LEG::LEG()
{
    
}
LEG::LEG(int in_index,int in_offset,int ot_index,int ot_offset)
{   

}

/// @brief 根据落脚点的位置， 计算 内外舵机的角度，
///        并将解算的角度， 直接替换参数 locale_x <-> inner_angle   locale_y <-> oouter_angle
///        由于这里先做的内容是， robot 的leg 开始和结束的点 都是 和地面接触的(不接触的写 另一个 trans函数， simple first)， 
///        所以 这里的两个自由度， 退化到 只剩一个自由度了，because z_0 = z_1 = 0
/// @param locale_x 
/// @param locale_y 
/// @param locale_z 
void LEG::trans_from_position_to_angle(int leg_index, float *locale_x, float *locale_y, float locale_z)// <---- todo  解算
{   
    // 这个trans 每条腿的结算不一样， 但如果有通用的 可以提出来
    switch (leg_index)
    {
    case LEG_0:
        /* code */

        break;

    case LEG_1:
        /* code */


        break;
    
    case LEG_2:
        /* code */
        break;
    case LEG_3:
        /* code */
        break;
    }
}

/// @brief 将 robot 下发的 位置坐标转为 角度坐标， 进而下发给舵机
/// @param leg_index 
/// @param locale_x 
/// @param locale_y 
/// @param locale_z 
void LEG::leg_exec(int leg_index, float locale_x, float locale_y, float locale_z)
{
    trans_from_position_to_angle(leg_index, &locale_x, &locale_y, locale_z); // 得到结算后的角度
    _in_motor_angle = locale_x;
    _ot_motor_angle = locale_y;
    
    MY_PCA9685_SET_ANGLE(_in_motor_index, _in_motor_angle - _in_motor_offset);
    MY_PCA9685_SET_ANGLE(_ot_motor_index, _ot_motor_angle - _ot_motor_offset); // 这里 分两次下发, 太难受了 <---- todo 改成一次下发 <--- 两次下发 好像也还ok
}

void LEG::set_leg(int in_index, int in_offset, int ot_index, int ot_offset)
{
    _in_motor_index = in_index;
    _in_motor_offset = in_offset;

    _ot_motor_index = ot_index;
    _ot_motor_offset = ot_offset;
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


    set_leg_rela(LEG_0, -HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH);
    set_leg_rela(LEG_1,  HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH);  // <--------------- Todo 
    set_leg_rela(LEG_2,  HALF_ROBOT_WIDTH,  HALF_ROBOT_LENGTH);
    set_leg_rela(LEG_3, -HALF_ROBOT_WIDTH,  HALF_ROBOT_LENGTH);

    reset();

    printf("init...\n");    
    
}


void MONKEY::set_leg_rela(int leg_index, float rela_x, float rela_y)
{
    assert(leg_index >=0 && leg_index <LEG_NUM);

    _leg_rela_x[leg_index] = rela_x;
    _leg_rela_y[leg_index] = rela_y;
}




/// @brief 
/*  

                leg1                    robot (HALF_ROBOT_WIDTH,  0,  0)
                  ____________________________.
                 /|
                / |
               /  |
        40    /   |   33
             /    |         arctan(20 / 33) = 30度
            /     |
           /      |
          /       |
         ---------|
        |    20
   20   |
        |
        |
        *  Foot
*/
void MONKEY::reset()
{
    //set_leg_global_position(0, )
}


/// @brief monkey 所有 leg 初始化
void MONKEY::reset()
{
    set_leg_position(LEG_0, -HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH, -53); // -53 = 20 + 40 * cos\theta

    set_leg_position(LEG_1,  HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH, -53); // -53 = 20 + 40 * cos\theta

    set_leg_position(LEG_2,  HALF_ROBOT_WIDTH,  HALF_ROBOT_LENGTH, -53); // -53 = 20 + 40 * cos\theta

    set_leg_position(LEG_3, -HALF_ROBOT_WIDTH,  HALF_ROBOT_LENGTH, -53); // -53 = 20 + 40 * cos\theta

    robot_exec();
}

void MONKEY::walk(float steps, int T = 500)
{
    printf("walk...\n");


}


/*


                （x，y, z）
                /
               /
              /
             /
           |/
           ```
          leg-1-upper-point (x - HALF-ROBOT-WIDTH, y + HALF-ROBOT-LENGTH, z)


*/

/// @brief Change the foot-global-relative-robot position to leg-upper-point-locale position.
///        把机器人 腿末端的， 相对机器人中心参考系的坐标， 转换为， 以那条腿和机器人连接点为局部坐标原点的 相对坐标
/// @param leg_index 
/// @param leg_global_x 
/// @param leg_global_y 
/// @param leg_global_z 
void MONKEY::trans_from_robot_to_leg(int leg_index, float *leg_global_x, float *leg_global_y, float *leg_global_z)
{
    assert(leg_index >=0 && leg_index < LEG_NUM);
    switch (leg_index)
    {
    case LEG_0:
        *leg_global_x = *leg_global_x + HALF_ROBOT_WIDTH;
        *leg_global_y = *leg_global_y + HALF_ROBOT_LENGTH;
        *leg_global_z = *leg_global_z;
        break;
    case LEG_1:
        *leg_global_x = *leg_global_x - HALF_ROBOT_WIDTH;
        *leg_global_y = *leg_global_y + HALF_ROBOT_LENGTH;
        *leg_global_z = *leg_global_z;
        break;

    case LEG_2:
        *leg_global_x = *leg_global_x - HALF_ROBOT_WIDTH;
        *leg_global_y = *leg_global_y - HALF_ROBOT_LENGTH;
        *leg_global_z = *leg_global_z;
        break;

    case LEG_3:
        *leg_global_x = *leg_global_x + HALF_ROBOT_WIDTH;
        *leg_global_y = *leg_global_y - HALF_ROBOT_LENGTH;
        *leg_global_z = *leg_global_z;
        break;
    
    default:
        break;
    }
}

void MONKEY::set_leg_position(int leg_index, float leg_global_x, float leg_global_y, float leg_global_z)
{   
    trans_from_robot_to_leg(leg_index, &leg_global_x, &leg_global_y, &leg_global_z);
    
    _leg_locale_position[leg_index][0] = leg_global_x;
    _leg_locale_position[leg_index][1] = leg_global_y;
    _leg_locale_position[leg_index][2] = leg_global_z;
}

/// @brief 机器人执行指令
void MONKEY::robot_exec()
{   
    // 四条腿执行陆续指令
    for(int i=0; i<LEG_NUM; ++i)
    {
        _leg[i].leg_exec(i, _leg_locale_position[i][0], _leg_locale_position[i][1], _leg_locale_position[i][2]);
    }
}

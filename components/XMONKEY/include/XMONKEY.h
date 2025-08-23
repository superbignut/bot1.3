/**
 * @file MONKEY.h
 * @author bignut
 * @brief
 * @version 0.1
 * @date 2025-08-5
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __LTL_XMONKEY__
#define __LTL_XMONKEY__

// #include "OSCILLATOR.h"


/*




    z
  ( . )----------->  y
    |
    |
    |
    |
    |
   \|/  x

                         y__
    x __                 |\     __ x
     |\                    \     /|
        0                   \   3
      /   \                  \/
    |/      4---------------7
   y```     |      Back     |
            |               |
            |     0,0,0     |
            |     Front     |        y
            5---------------6     ``/|
          /                   \    /
       1                        2 /
      / \                         \
    /    \                       __\|
  |/__    \                          x
  x       _\| y




inner motor:  4-5-6-7 is rotate around z axis.
outer motor:  0-1-2-3 is rotate around x's relative axis


*/

#define LEG_NUM 4
#define MOTOR_NUM (LEG_NUM * 2)

class INNER_MOTOR
{

public:
    INNER_MOTOR();
    INNER_MOTOR(int leg_index, int offset);

    void set_leg_index(int index);

    void set_leg_offset(int offset);

private:
    void set_motor_angle(float angle);



private:
    int _leg_index = 5;
    int _offset = 0;
};

class OUTER_MOTOR
{
public:
    OUTER_MOTOR();
    OUTER_MOTOR(int leg_index, int offset);

    void set_leg_index(int index);

    void set_leg_offset(int offset);

private:
    void set_motor_angle(float angle);

private:
    int _leg_index = 5;
    int _offset = 0;
};

class LEG
{
public:
    LEG();

    LEG(int in_index,int in_offset,int ot_index,int ot_offset); // 初始化电机编号， 与位置补偿

    void set_locale_position(float locale_x, float locale_y, float locale_z);    // 设置位置，当前参考系

private:
    float* from_position_to_angle(float locale_x, float locale_y, float locale_z);// 将locale地址 转为角度坐标, 解算

    void set_angle_position(float in_angle, float ot_angle);  // 设置角度， 下发 motor

    // void 
private:
    
    float current_motor_pos[MOTOR_NUM / LEG_NUM]; // 当下电机的位置

    INNER_MOTOR in_motor;   // 内侧电机
    OUTER_MOTOR ot_motor;   // 外侧电机
};

class MONKEY
{
public:
    MONKEY(); // 初始化

    void reset();   // 立正了  <--------------Todo

    void walk(float steps, int period); // 进入步态大小循环， 计算robot 坐标下 每个腿末态位置， 不进行接算

    void from_robot_to_leg(); //  将robot 坐标转为 腿坐标

    void set_leg_global_position(int leg_index, float leg_global_x, float leg_global_y, float leg_global_z);    // 将全局坐标转换为相对， 并下发给leg

private:

    LEG leg[LEG_NUM];

    float leg_rela_x[LEG_NUM];  // 每个腿 相对 robot 中心的位置坐标
    float leg_rela_y[LEG_NUM];
    
    float global_position[3];   // robot's xyz

    // float global_leg_position[LEG_NUM][3];  // 记录 腿 终点（与地面接触点） 的坐标
};

#endif
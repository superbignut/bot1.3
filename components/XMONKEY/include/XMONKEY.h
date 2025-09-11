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


   leg0                             leg3
                         y__
    x __                 |\     __ x
     |\                    \     /|
        0                   \   3
      /   \                  \/
    |/      4---------------7
   y```     |      Back     |
            |               |
            |     0,0,0     |  width
            |               |
            |     Front     |        y
            5---------------6     ``/|
          /      length       \    /
       1                        2 /
      / \                         \
    /    \                       __\|
  |/__    \                          x
  x       _\| y

    leg1                                leg2


inner motor:  4-5-6-7 is rotate around z axis.
outer motor:  0-1-2-3 is rotate around x's relative axis


*/

#define LEG_NUM 4
#define MOTOR_NUM (LEG_NUM * 2)

#define LEG_0 0
#define LEG_1 1
#define LEG_2 2
#define LEG_3 3

#define ROBOT_LENGTH (80.6)
#define ROBOT_WIDTH (67.5)
#define ROBOT_HIGH (30)

#define HALF_ROBOT_LENGTH (80.6 / 2)
#define HALF_ROBOT_WIDTH (67.5 / 2)
#define HALF_ROBOT_HIGH (30 / 2)

#define LEG_L1 (25.75)
#define LEG_L2 (40.0)
#define LEG_L3 (20.0)

// 这个角度可以设置 monkey的 初始高度
#define LEG_IN_BETA_0 (10.0 / 180.0 * M_PI)     

#define ANGLE_2_RAD(x) ((x) * (3.1415 / 180.0))
#define RAD_2_ANGLE(x) ((x) * (180.0 / 3.1415))

#define INIT_MONKEY_H (LEG_L3 + LEG_L2 * cos(LEG_IN_BETA_0))

#define MOTOR_0 0
#define MOTOR_1 1
#define MOTOR_2 2
#define MOTOR_3 3
#define MOTOR_4 4
#define MOTOR_5 5
#define MOTOR_6 6
#define MOTOR_7 7

#define STEPS_PER_PART 100

enum MONKEY_STATUS 
{
    X_RESET=0, X_WALK_F, X_WALK_B, X_ROTATE_L, X_ROTATE_R, X_TEST
};


class LEG
{
public:
    LEG();

    LEG(int leg_index, int in_index,int in_offset,int ot_index,int ot_offset); // 初始化电机编号， 与位置补偿

    void leg_exec(float locale_x, float locale_y, float locale_z);    // 设置位置，当前参考系

    void set_leg(int leg_index, int in_index,int in_offset,int ot_index,int ot_offset);



private:
    
    void trans_from_position_to_angle(float *locale_x, float *locale_y, float locale_z);// 将locale地址 转为角度坐标, 解算

    void set_angle_position(float in_angle, float ot_angle);  // 设置角度， 下发 motor

    float convert_angle_to_9685_angle(int motor_index, float angle);

private:
    
    // float current_motor_pos[MOTOR_NUM / LEG_NUM]; // 当下电机的位置

    int _in_motor_index = 5;
    float _in_motor_offset = 0;
    float _in_motor_angle;  // Radian

    int _ot_motor_index = 5;
    float _ot_motor_offset = 0;
    float _ot_motor_angle;
    
    int _leg_index;
};

class MONKEY
{
public:
    MONKEY(); // 初始化

    void main_loop();

    void set_status(MONKEY_STATUS s);

private:

    void test();

    void reset();   // 初始化  <--------------Todo

    void walk(float steps, int period); // 进入步态大小循环， 计算robot 坐标下 每个腿末态位置， 不进行接算

    void trans_from_robot_to_leg(int leg_index, float *leg_global_x, float *leg_global_y, float *leg_global_z); //  将robot 坐标转为 腿坐标

    void set_leg_position(int leg_index, float leg_x, float leg_y, float leg_z);    // 将全局坐标转换为相对， 并下发给leg

    void robot_exec();    // 运动下发

    void set_leg_link_point(int leg_index, float rela_x, float rela_y);   // 设置 leg 相对 body 的坐标

    void creep();

    LEG _leg[LEG_NUM];

    float _leg_link_body_x[LEG_NUM];  // 每个腿 相对 robot 中心的位置坐标

    float _leg_link_body_y[LEG_NUM];
    
    float _robot_global_position[3];   // robot's xyz 

    float _leg_locale_position[LEG_NUM][3];

    MONKEY_STATUS _robot_status;    // 使用这个全局状态的切换来 控制不同的步态 

    // float global_leg_position[LEG_NUM][3];  // 记录 腿 终点（与地面接触点） 的坐标
};

#endif
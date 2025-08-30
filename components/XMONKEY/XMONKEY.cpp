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
#include "cmath"

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
///        这里对更一般的 z 存在高度的情况进行解算
///        x 解算一个自由度 -> inner_angle  z解算另一个自由度 -> ot_angle   
///        
/// @param locale_x theta'
/// @param locale_y 
/// @param locale_z 
void LEG::trans_from_position_to_angle(int leg_index, float *locale_x, float *locale_y, float locale_z)// <---- todo  解算
{   

    /*  leg0                                                           leg3
        +-----------+-----------4---------------7-----------+-----------+
                           θ (/ |      Back     |\ θ
                             /  |               | \
                            /   |               |  \
                                |               |
         leg_l2*sin     leg_l1  |     Front     |        
        +-----------+-----------5---------------6------------+----------+
        leg1                θ (/                 \  θ                  leg2
                              /                   \
                             /                     \
                            /               

            -------------------> y
            |
            |
            |
            |
            |
            |
           \|/
            x
        这里解算的 都是这里的 θ

        
        x' = (l_1 + l_2 * sin\beta')sin\theta' 
        y' = -(l_1 + l_2 * sin\beta')cos\theta'  ---> leg1

        这里 由于只有一个自由度, 所以用 x y 中的一个就可以了

        l_2 * cos( beta_0 ) + l3 = l_2 * cos( beta' ) + l3 + h   <---- 这里可以解算出 抬腿的高度,



        但如果 h = 0 : 则 \beta' = \beta_0 = 30度

        \theta' = new angle 这里就是要解算的仅剩的自由度了
        
    */


    // 这个trans 每条腿的结算不一样， 但如果有通用的 可以提出来

    /*  

                leg1         l1         robot (HALF_ROBOT_WIDTH,  0,  0)
                  .___________________________.
                 /|     leg-body link point
                / | β
           l2  /  |
        40    /   |   34.6
             /    |         arctan(20 / 33) = 30度
            /     |
           /      |
          /       |
         ---------|
        |    20
   20   |
        | l3
        |
        *  Foot

        其余 leg 的 β 同理， 向内收为减小， 向外张为 增大
*/

    assert(locale_z >= 0);

    float theta;
    float beta;
    
    switch (leg_index)
    {
    case LEG_0:
        /* code */
        
        theta = asin(*locale_x / (LEG_L1 + LEG_L2 * sin(LEG_IN_BETA_0)));

        *locale_x = theta;
        break;

    case LEG_1:

        theta = asin(*locale_x / (LEG_L1 + LEG_L2 * sin(LEG_IN_BETA_0)));

        *locale_x = theta;
        
        break;
    
    case LEG_2:
        /* code */
        theta = asin(*locale_x / (LEG_L1 + LEG_L2 * sin(LEG_IN_BETA_0)));
        
        *locale_x = theta;
        break;
    case LEG_3:
        /* code */
        theta = asin(*locale_x / (LEG_L1 + LEG_L2 * sin(LEG_IN_BETA_0)));
                        
        *locale_x = theta;
        break;
    }

    // 解算 beta 角
    beta = acos(LEG_L2 * cos(LEG_IN_BETA_0) - locale_z);

    *locale_y = beta;
    
}


/// @brief 根据落脚点的位置， 计算 内外舵机的角度，
///        并将解算的角度， 直接替换参数 locale_x <-> inner_angle   locale_y <-> oouter_angle
///        由于这里先做的内容是， robot 的leg 开始和结束的点 都是 和地面接触的(不接触的写 另一个 trans函数， simple first)，
///        也就是 z_0 = z_1 = 0
///        所以 这里的两个自由度， 退化到 只剩一个自由度了，because z_0 = z_1 = 0
/// @param locale_x theta'
/// @param locale_y 
/// @param locale_z 
void LEG::trans_from_position_to_angle_z0(int leg_index, float *locale_x, float *locale_y, float locale_z)// <---- todo  解算
{   

    /*  leg0                                                           leg3
        +-----------+-----------4---------------7-----------+-----------+
                           θ (/ |      Back     |\ θ
                             /  |               | \
                            /   |               |  \
                                |               |
         leg_l2*sin     leg_l1  |     Front     |        
        +-----------+-----------5---------------6------------+----------+
        leg1                θ (/                 \  θ                  leg2
                              /                   \
                             /                     \
                            /               

            -------------------> y
            |
            |
            |
            |
            |
            |
           \|/
            x
        这里解算的 都是这里的 θ

        
        x' = (l_1 + l_2 * sin\beta')sin\theta' 
        y' = -(l_1 + l_2 * sin\beta')cos\theta'  ---> leg1

        这里 由于只有一个自由度, 所以用 x y 中的一个就可以了

        l_2 * cos( beta_0 ) + l3 = l_2 * cos( beta' ) + l3 + h   <---- 这里可以解算出 抬腿的高度,

        但如果 h = 0 : 则 \beta' = \beta_0 = 30度

        \theta' = new angle 这里就是要解算的仅剩的自由度了
        
    */


    // 这个trans 每条腿的结算不一样， 但如果有通用的 可以提出来

    assert(locale_z == 0);

    float theta;

    float beta;
    
    switch (leg_index)
    {
    case LEG_0:
        /* code */
        
        theta = asin(*locale_x / (LEG_L1 + LEG_L2 * sin(LEG_IN_BETA_0)));

        *locale_x = theta;
        break;

    case LEG_1:

        theta = asin(*locale_x / (LEG_L1 + LEG_L2 * sin(LEG_IN_BETA_0)));

        *locale_x = theta;
        
        break;
    
    case LEG_2:
        /* code */
        theta = asin(*locale_x / (LEG_L1 + LEG_L2 * sin(LEG_IN_BETA_0)));
        
        *locale_x = theta;
        break;
    case LEG_3:
        /* code */
        theta = asin(*locale_x / (LEG_L1 + LEG_L2 * sin(LEG_IN_BETA_0)));
                        
        *locale_x = theta;
        break;
    }

    // *locale_y = LEG_IN_BETA_0; // 这里直接赋给 初始值


    beta = acos((LEG_L2 * cos(LEG_IN_BETA_0) - locale_z) / LEG_L2);

    *locale_y = beta;
}

/// @brief 将解算的期望 angle 转为 pca9685 的 angle 之间需要一个转换, 与实际的舵机安装位置、朝向有关
/// @param leg_index 
/// @param angle  解算出的 角度 angle : 0-180
/// @return 舵机能处理的角度
float LEG::convert_angle_to_9685_angle(int motor_index, float angle) // <- int motor_index <-----Todo
{
    if(motor_index == MOTOR_4 || motor_index == MOTOR_5)      // <----- 这里不是和 leg 的关系, 而是和 motor 的关系
    {
        return 90.0 - angle;
    }
    else if (motor_index == MOTOR_6 || motor_index == MOTOR_7)
    {
        return angle + 90.0;
    }
    else if (motor_index == MOTOR_0 || motor_index == MOTOR_1)
    {
        return 90.0 - angle;
    }
    else if (motor_index == MOTOR_2 || motor_index == MOTOR_3)
    {
        return 90.0 + angle;
    }
    return angle;
}
/// @brief 将 robot 下发的 位置坐标转为 角度坐标， 进而下发给舵机
/// @param leg_index 
/// @param locale_x 
/// @param locale_y 
/// @param locale_z 
void LEG::leg_exec(int leg_index, float locale_x, float locale_y, float locale_z)
{   
    // printf("LEG POS IS: %d, x: %.2f, y: %.2f, z: %.2f\n", leg_index, locale_x, locale_y, locale_z);
    // return 
    if(locale_z == 0)
    {

        
        trans_from_position_to_angle_z0(leg_index, &locale_x, &locale_y, 0); // 得到结算后的角度

    }
    else 
    {
        trans_from_position_to_angle(leg_index, &locale_x, &locale_y, locale_z);
    }

        /* printf("LEG ANGLE IS: %d, x: %.2f, y: %.2f, z: %.2f\n", leg_index, locale_x, locale_y, locale_z);
        return; */

        _in_motor_angle = convert_angle_to_9685_angle(_in_motor_index, RAD_2_ANGLE(locale_x));
        _ot_motor_angle = convert_angle_to_9685_angle(_ot_motor_index, RAD_2_ANGLE(locale_y));


        MY_PCA9685_SET_ANGLE(_in_motor_index, _in_motor_angle);

        printf("In_Motor INDEX IS : %d, inner angle: %.2f, outer angle: %.2f \n", _in_motor_index, _in_motor_angle, _ot_motor_angle);
        // return;
        // printf("this is --> %.2f", locale_x);
        
        MY_PCA9685_SET_ANGLE(_ot_motor_index, _ot_motor_angle); // 这里 分两次下发, 太难受了 <---- todo 改成一次下发 <--- 两次下发 好像也还ok
    
    
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
    // MY_PCA9685_Init();
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


/// @brief 这里是设置 leg-body-link-point 相对位置
/// @param leg_index 
/// @param rela_x 
/// @param rela_y 
void MONKEY::set_leg_rela(int leg_index, float rela_x, float rela_y)
{
    assert(leg_index >=0 && leg_index <LEG_NUM);

    _leg_rela_x[leg_index] = rela_x;
    _leg_rela_y[leg_index] = rela_y;
}




/// @brief 
/*  

                leg1         l1         robot (HALF_ROBOT_WIDTH,  0,  0)
                  .___________________________.
                 /| leg-body link point
                / |
           l2  /  |
        40    /   |   34.6
             /    |         arctan(20 / 33) = 30度
            /     |
           /      |
          /       |
         ---------|
        |    20
   20   |
        | l3
        |
        *  Foot
*/




/// @brief monkey 所有 leg 初始化
///        这里其实是把所有的 leg-body-link-point 在水平面的投影作为的基准点
void MONKEY::reset()
{
    /*
                                4---------------7
                                |      Back     |
                                |               |
                                |     0,0,h     |
                                |               |
         leg_l2*sin     leg_l1  |     Front     |        
        +-----------+-----------5---------------6
    
    
    */
    int tmp_l = LEG_L1 + LEG_L2 * sin(LEG_IN_BETA_0);

    set_leg_position(LEG_0, -HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH - tmp_l, 0); // -53 = 20 + 40 * cos\theta

    set_leg_position(LEG_1,  HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH - tmp_l, 0); // -53 = 20 + 40 * cos\theta

    set_leg_position(LEG_2,  HALF_ROBOT_WIDTH,  HALF_ROBOT_LENGTH + tmp_l, 0); // -53 = 20 + 40 * cos\theta

    set_leg_position(LEG_3, -HALF_ROBOT_WIDTH,  HALF_ROBOT_LENGTH + tmp_l, 0); // -53 = 20 + 40 * cos\theta

    robot_exec();
}


void MONKEY::test()
{
    
    int l = 10;

    do
    {
        l = 20;

        set_leg_position(LEG_0, -HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_1,  HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_2,  HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_3, -HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        robot_exec();

        vTaskDelay(500 / portTICK_PERIOD_MS);

        l = 0;

        set_leg_position(LEG_0, -HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_1,  HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_2,  HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_3, -HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        robot_exec();

        vTaskDelay(500 / portTICK_PERIOD_MS);


    }while(false);


}


void MONKEY::test_z()
{
    
    int l = 10;

    do
    {
        l = 20;

        set_leg_position(LEG_0, -HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_1,  HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_2,  HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_3, -HALF_ROBOT_WIDTH + l, 0, 0); // -53 = 20 + 40 * cos\theta

        robot_exec();

        vTaskDelay(500 / portTICK_PERIOD_MS);


    } while(false);


}

void MONKEY::walk(float steps, int T = 500)
{
    printf("walk...\n");
    //                       <------ Todo
    // 这里应该是计算出每条腿 xyz 然后分别执行
    //
    
    while(true)
    {
        // 给出part 1 四条腿的目标位置
        // part 1 比如是100个 分段来执行第一个 part
        for(int i=0; i< 100; ++i)
        {
            // 计算 从初始位置, 到目标位置 四条腿的插值
            // 四条腿分别执行
            // _leg[i].leg_exec(i, )
        }

        // part 2
        // 进入下一个 part
        
    }
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

/// @brief 设置腿的全局位置, 函数内会将 全局位置转换为 locale 位置 存在成员变量 _leg_locale_position 中
///        进而需要 进一步执行 robot_exec 才能运动, 或者可以考虑将 robot_exec 收纳进来
///        所有的 leg-body-link-point 在水平面的投影作为 local 的基准点
/// @param leg_index 
/// @param leg_global_x 
/// @param leg_global_y 
/// @param leg_global_z 
void MONKEY::set_leg_position(int leg_index, float leg_global_x, float leg_global_y, float leg_global_z)
{   
    trans_from_robot_to_leg(leg_index, &leg_global_x, &leg_global_y, &leg_global_z);
    
    _leg_locale_position[leg_index][0] = leg_global_x;
    _leg_locale_position[leg_index][1] = leg_global_y;
    _leg_locale_position[leg_index][2] = leg_global_z;
    
    // robot_exec()
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

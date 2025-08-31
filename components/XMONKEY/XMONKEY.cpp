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



LEG::LEG()
{
}
LEG::LEG(int in_index, int in_offset, int ot_index, int ot_offset)
{
}

/// @brief 根据落脚点的位置， 计算 内外侧 舵机的角度，并将解算的角度， 直接替换输入指针参数
///        locale_x <-> inner_angle   locale_y <-> oouter_angle
///        这里是对更一般的 z， 也即存在抬腿高度的情况进行解算
///        x 可以解算一个自由度 -> inner_angle, z 解算另一个自由度 -> ot_angle
///
/// @param locale_x theta'
/// @param locale_y 返回的是弧度
/// @param locale_z
void LEG::trans_from_position_to_angle(int leg_index, float *locale_x, float *locale_y, float locale_z) // <---- todo  解算
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

    // _in_motor_angle 内侧舵机角度
    float theta;

    // _ot_motor_angle 外侧舵机角度
    float beta;

    // 解算 theta
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
    beta = acos((LEG_L2 * cos(LEG_IN_BETA_0) - locale_z) / LEG_L2);

    *locale_y = beta;
}


/*

    trans_from_position_to_angle_z0 已废弃，最初设计这个函数的时候，是为了逐步解算， 先把 ot_motor 固定住
    
    只关心 in_motor， 目前已实现 trans_from_position_to_angle 为最新 trans函数

    由于存在早期注释，暂不删去次函数
*/

/// @brief 根据落脚点的位置， 计算 内外舵机的角度，
///        并将解算的角度， 直接替换参数 locale_x <-> inner_angle   locale_y <-> oouter_angle
///        由于这里先做的内容是， robot 的leg 开始和结束的点 都是 和地面接触的(不接触的写 另一个 trans函数， simple first)，
///        也就是 z_0 = z_1 = 0
///        所以 这里的两个自由度， 退化到 只剩一个自由度了，because z_0 = z_1 = 0
/// @param locale_x theta'
/// @param locale_y 返回的是弧度
/// @param locale_z
void LEG::trans_from_position_to_angle_z0(int leg_index, float *locale_x, float *locale_y, float locale_z) // <---- todo  解算
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
    if (motor_index == MOTOR_4 || motor_index == MOTOR_5) // <----- 这里不是和 leg 的关系, 而是和 motor 的关系
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

        trans_from_position_to_angle(leg_index, &locale_x, &locale_y, locale_z);

    /* printf("LEG ANGLE IS: %d, x: %.2f, y: %.2f, z: %.2f\n", leg_index, locale_x, locale_y, locale_z);
    return; */

    /* printf("Origin : In_Motor INDEX IS : %d, inner angle: %.2f, Ot_Motor INDEX IS: %d, outer angle: %.2f \n", \
                    _in_motor_index, RAD_2_ANGLE(locale_x), _ot_motor_index, RAD_2_ANGLE(locale_y)); */

    _in_motor_angle = convert_angle_to_9685_angle(_in_motor_index, RAD_2_ANGLE(locale_x));
    _ot_motor_angle = convert_angle_to_9685_angle(_ot_motor_index, RAD_2_ANGLE(locale_y));

    /* printf("In_Motor INDEX IS : %d, inner angle: %.2f, Ot_Motor INDEX IS: %d, outer angle: %.2f \n", \
                        _in_motor_index, _in_motor_angle, _ot_motor_index, _ot_motor_angle); */

    MY_PCA9685_SET_ANGLE(_in_motor_index, _in_motor_angle);
    MY_PCA9685_SET_ANGLE(_ot_motor_index, _ot_motor_angle); 
}

/// @brief 设置每条腿上 舵机的编号和偏移量
/// @param in_index  编号用来区分后续的解算， 因为不同位置的舵机解算应该是不同的
/// @param in_offset 偏移量暂未使用， 主要是为了修正 安装时的角度初始偏差
/// @param ot_index 
/// @param ot_offset 
void LEG::set_leg(int in_index, int in_offset, int ot_index, int ot_offset)
{
    _in_motor_index = in_index;
    _in_motor_offset = in_offset;

    _ot_motor_index = ot_index;
    _ot_motor_offset = ot_offset;
}

/// @brief robot 初始化机器人参数: 舵机编号、偏移、连接点坐标， 并初始化姿态
MONKEY::MONKEY()
{
    
    //LEG init, 注册编号和 offset
    _leg[0].set_leg(4, 0, 0, 0); // leg 初始化 编号和offset
    _leg[1].set_leg(5, 0, 1, 0); // leg 初始化 编号和offset
    _leg[2].set_leg(6, 0, 2, 0); // leg 初始化 编号和offset
    _leg[3].set_leg(7, 0, 3, 0); // leg 初始化 编号和offset

    // 设置每条腿和身体连接点的位置坐标， 以 body 中心为原点
    set_leg_link_point(LEG_0, -HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH);
    set_leg_link_point(LEG_1, HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH); // <--------------- Todo
    set_leg_link_point(LEG_2, HALF_ROBOT_WIDTH, HALF_ROBOT_LENGTH);
    set_leg_link_point(LEG_3, -HALF_ROBOT_WIDTH, HALF_ROBOT_LENGTH);

    // 舵机初始化
    reset();

    printf("init...\n");
}

/// @brief 这里是设置 leg-body-link-point 的相对位置， 以机器人的 body 中心为原点
/// @param leg_index
/// @param rela_x
/// @param rela_y
void MONKEY::set_leg_link_point(int leg_index, float rela_x, float rela_y)
{
    assert(leg_index >= 0 && leg_index < LEG_NUM);

    _leg_link_body_x[leg_index] = rela_x;
    _leg_link_body_y[leg_index] = rela_y;
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

    if (_robot_status == X_RESET)
    {

        printf("%d\n", _robot_status);

        // 由于使用投影点作为参考点， 所以这里的 z轴 初始化是0, 比较容易误解
        // l_2 * cos( beta_0 ) + l3 = l_2 * cos( beta' ) + l3 + h
        // 上式为姿态解算 ot_motor 的表达式， 其中 h 为抬腿高度， 可以辅助理解
        // 结果为: 当 h 增大时， 机器人的高度下降
        set_leg_position(LEG_0, -HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH - tmp_l, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_1, HALF_ROBOT_WIDTH, -HALF_ROBOT_LENGTH - tmp_l, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_2, HALF_ROBOT_WIDTH, HALF_ROBOT_LENGTH + tmp_l, 0); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_3, -HALF_ROBOT_WIDTH, HALF_ROBOT_LENGTH + tmp_l, 0); // -53 = 20 + 40 * cos\theta

        robot_exec();

        // printf("MONKEY STATUS: RESET!\n");

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/// @brief 测试函数
void MONKEY::test()
{

    int l = 10;
    int h = 10;

    if (_robot_status == X_TEST)
    {
        l = 20;
        h = 10;

        set_leg_position(LEG_0, -HALF_ROBOT_WIDTH + l, 0, h); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_1, HALF_ROBOT_WIDTH + l, 0, h); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_2, HALF_ROBOT_WIDTH + l, 0, h); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_3, -HALF_ROBOT_WIDTH + l, 0, h); // -53 = 20 + 40 * cos\theta

        robot_exec();

        vTaskDelay(500 / portTICK_PERIOD_MS);

        l = 0;
        h = 0;

        set_leg_position(LEG_0, -HALF_ROBOT_WIDTH + l, 0, h); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_1, HALF_ROBOT_WIDTH + l, 0, h); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_2, HALF_ROBOT_WIDTH + l, 0, h); // -53 = 20 + 40 * cos\theta

        set_leg_position(LEG_3, -HALF_ROBOT_WIDTH + l, 0, h); // -53 = 20 + 40 * cos\theta

        robot_exec();

        vTaskDelay(500 / portTICK_PERIOD_MS);

    }
}

/// @brief 机器人进入 this->main 函数后，使用全局变量 _robot_status 切换状态
/// @param s 
void MONKEY::set_status(MONKEY_STATUS s)
{
    _robot_status = s;
}

/// @brief robot 主函数， 负责切换不同的步态
void MONKEY::main()
{
    while (true)
    {
        
        switch (_robot_status)
        {
        case X_RESET:
            reset();
            break;
        case X_WALK_F:
            walk(0, 0);
            break;

        case X_WALK_B:
            walk(0, 0);
            break;

        case X_TEST:
            test();
            break;

        case X_ROTATE_L:
        case X_ROTATE_R:

        default:
            reset();
            break;
        }
    }
}

/// @brief 机器人 行走步态，暂未实施
/// @param steps 
/// @param T 
void MONKEY::walk(float steps, int T = 500)
{
    printf("walk...\n");
    //                       <------ Todo
    // 这里应该是计算出每条腿 xyz 然后分别执行
    //

    while (true)
    {
        // 给出part 1 四条腿的目标位置
        // part 1 比如是100个 分段来执行第一个 part
        for (int i = 0; i < 100; ++i)
        {
            // 计算 从初始位置, 到目标位置 四条腿的插值
            // 四条腿分别执行
            // _leg[i].leg_exec(i, )
        }

        // part 2
        // 进入下一个 part
    }
}


/// @brief 把机器人foot 的，相对机器人中心参考系的全局坐标，转换为，以leg 和body 连接点为原点的 相对坐标
///        这里的设计目的就是，所有 leg 的目的坐标均使用以 robot中心为原点的全局坐标
///        因此在机器人层面规划运动时， 也是使用统一的全局坐标，算是一种约定吧
/// @param leg_index
/// @param leg_global_x 参数都是指针类型，转换的结果保存在参数中
/// @param leg_global_y
/// @param leg_global_z
void MONKEY::trans_from_robot_to_leg(int leg_index, float *leg_global_x, float *leg_global_y, float *leg_global_z)
{
    assert(leg_index >= 0 && leg_index < LEG_NUM);

    /*
        leg_global - leg_link_body = leg_rela_link

        leg落脚点的全局坐标 - leg-body 连接点的坐标 = leg落脚点相对连接点的坐标


                     robot_center
                         /\
                        /  \
                       /    \
          leg_global  /      \  leg_link_body
                     /        \
                    /          \|
                   /          ``` 
                  /          /    
                 /        /   
                /      /     
               /     /      leg_rela_link
              /    /        
            |/  </
            ```
          foot

    */
   
    // 不同的leg 是不同的 link-point
    switch (leg_index)
    {
    case LEG_0:
        *leg_global_x = *leg_global_x - _leg_link_body_x[LEG_0];
        *leg_global_y = *leg_global_y - _leg_link_body_y[LEG_0];
        *leg_global_z = *leg_global_z;
        break;
    case LEG_1:
        *leg_global_x = *leg_global_x - _leg_link_body_x[LEG_1];
        *leg_global_y = *leg_global_y - _leg_link_body_x[LEG_1];
        *leg_global_z = *leg_global_z;
        break;

    case LEG_2:
        *leg_global_x = *leg_global_x - _leg_link_body_x[LEG_2];
        *leg_global_y = *leg_global_y - _leg_link_body_x[LEG_2];
        *leg_global_z = *leg_global_z;
        break;

    case LEG_3:
        *leg_global_x = *leg_global_x - _leg_link_body_x[LEG_3];
        *leg_global_y = *leg_global_y - _leg_link_body_x[LEG_3];
        *leg_global_z = *leg_global_z;
        break;

    default:
        break;
    }
}

/// @brief 设置腿的全局位置, 函数内的 trans 会将 全局位置转换为 locale 位置 存于成员变量 _leg_locale_position 中
///        进而需要 继续执行 robot_exec 才能完成运动下发, 或者可以考虑将 robot_exec 收纳进来
///        以 leg-body-link-point 在水平面的投影作为 local 的基准点
/// @param leg_index
/// @param leg_global_x
/// @param leg_global_y
/// @param leg_global_z
void MONKEY::set_leg_position(int leg_index, float leg_global_x, float leg_global_y, float leg_global_z)
{
    // 转换: 之所以使用引用是把返回值 直接 对参数进行了替换
    trans_from_robot_to_leg(leg_index, &leg_global_x, &leg_global_y, &leg_global_z);

    // 转换结果保存起来， 其实如果把 robot_exec 吸收进来， 就不需要这个成员变量了 <-------- Todo
    _leg_locale_position[leg_index][0] = leg_global_x;
    _leg_locale_position[leg_index][1] = leg_global_y;
    _leg_locale_position[leg_index][2] = leg_global_z;

    // robot_exec()
}

/// @brief 机器人执行指令， 四肢分别执行
void MONKEY::robot_exec()
{
    // 四条腿执行陆续指令
    for (int i = 0; i < LEG_NUM; ++i)
    {
        _leg[i].leg_exec(i, _leg_locale_position[i][0], _leg_locale_position[i][1], _leg_locale_position[i][2]);
    }
}

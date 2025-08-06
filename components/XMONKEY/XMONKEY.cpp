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
#include "PCA9685.h"
#include "stdio.h"


void MONKEY::init()
{   
    MY_PCA9685_Init();

    printf("init...\n");    
    
    // Trim values for zero position calibration.
    trim[0] = 0;
    trim[1] = 0;
    trim[2] = 0;
    trim[3] = 0;
    trim[4] = 0;
    trim[5] = 0;
    trim[6] = 0;
    trim[7] = 0;

    // For the reason of mirror-position of servo. Whether reverse canbe used to control Servo.
    for (int i=0; i<LEG_NUM; ++i)
    {
        reverse[i] = false;
    }
        
    for(int i=0; i<LEG_NUM; ++i)
    {
        oscillator[i].start();
    }

    zero();
}
void MONKEY::run(float steps, int period)
{
}
void MONKEY::walk(float steps, int period)
{
    printf("walk...\n");
}
void MONKEY::omniWalk(float steps, int T, bool side, float turn_factor)
{
}

void MONKEY::turnL(float steps, int period)
{
}
void MONKEY::turnR(float steps, int period)
{
}
void MONKEY::moonwalkL(float steps, int period)
{
}
void MONKEY::dance(float steps, int period)
{
}
void MONKEY::upDown(float steps, int period)
{
}
void MONKEY::pushUp(float steps, int period)
{
}
void MONKEY::hello()
{
}
void MONKEY::jump()
{
}
void MONKEY::home()
{
}



static void set_servo_angle(int pin, float angle)
{    
    setPWM(pin, 0, PCA_9685_Angle_to_Num(angle));
}

/// @brief Set all leg to *Zero* position.
void MONKEY::zero()
{
    for(int i=0; i < LEG_NUM; ++i){
        set_servo_angle(i, 90);
    }
}
void MONKEY::frontBack(float steps, int period)
{
}

void MONKEY::setServo(int id, float target)
{
}
void MONKEY::reverseServo(int id)
{
}
float getServo(int id)
{
    return 1.0;
}
void MONKEY::moveServos(int time, float target[8])
{
}

int angToUsec(float value)
{
    return 0;
}
void MONKEY::execute(float steps, int period[8], int amplitude[8], int offset[8], int phase[8])
{
}

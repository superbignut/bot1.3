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

#include "OSCILLATOR.h"


#define LEG_NUM 8

class MONKEY{
public:
    void init();
    void run(float steps, int period);
    void walk(float steps, int period);
    void omniWalk(float steps, int T, bool side, float turn_factor);
    //void backward(float steps, int period);
    void turnL(float steps, int period);
    void turnR(float steps, int period);
    void moonwalkL(float steps, int period);
    void dance(float steps, int period);
    void upDown(float steps, int period);
    void pushUp(float steps, int period);
    void hello();
    void jump();
    void home();
    void zero();
    void frontBack(float steps, int period);

    void setServo(int id, float target);
    void reverseServo(int id);
    float getServo(int id);
    void moveServos(int time, float target[8]);

private:
    Oscillator oscillator[LEG_NUM];
    // Servo servo[8];
    // int board_pins[LEG_NUM];
    int trim[LEG_NUM];
    bool reverse[LEG_NUM];
    unsigned long _init_time;
    unsigned long _final_time;
    unsigned long _partial_time;
    float _increment[LEG_NUM];
    float _servo_position[LEG_NUM];

    int angToUsec(float value);
    
    void execute(float steps, int period[LEG_NUM], int amplitude[LEG_NUM], int offset[LEG_NUM], int phase[LEG_NUM]);
};

#endif
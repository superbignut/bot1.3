/**
 * @file OSCILLATOR.c
 * @author bignut
 * @brief 
 * @version 0.1
 * @date 2025-07-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdio.h>
#include "OSCILLATOR.h"
#include "esp_timer.h"
#include "math.h"

Oscillator::Oscillator(){
    esp_timer_early_init();
    esp_timer_init(); // init

    _period = 2000;
    _amplitude = 50;
    _phase = 0;
    _offset = 0;
    _stop = true;
    _ref_time = esp_timer_get_time();
    _delta_time = 0;
}

float Oscillator::refresh(){
    if (!_stop){
        _delta_time = (esp_timer_get_time()-_ref_time) % _period;
        _output =   (float)_amplitude*sin(time_to_radians(_delta_time)
                    + degrees_to_radians(_phase))
                    + _offset;
    }

    return _output;
}
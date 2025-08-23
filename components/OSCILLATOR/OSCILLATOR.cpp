/* 
#include <stdio.h>
#include "OSCILLATOR.h"
#include "esp_timer.h"
#include "math.h"

Oscillator::Oscillator()
{
    // esp_timer_early_init();
    // esp_timer_init(); // init

    _period = 2000;
    _amplitude = 50;
    _phase = 0;
    _offset = 0;
    _stop = true;
    _ref_time = esp_timer_get_time();
    _delta_time = 0;
}

/// @brief A * sin(dt / period + _phi )  + offset
///        Update the sin output if not stop;
/// @return
float Oscillator::refresh()
{
    if (!_stop)
    {
        _delta_time = (esp_timer_get_time() - _ref_time) % _period;
        _output = (float)_amplitude * sin(time_to_radians(_delta_time) + degrees_to_radians(_phase)) + _offset;
    }

    return _output;
}


/// @brief Update _ref_time
void Oscillator::reset()
{
    _ref_time = esp_timer_get_time();
}

/// @brief Stop = False, Update _refer_time
void Oscillator::start()
{
    _stop = false;   
    _ref_time = esp_timer_get_time();
}

/// @brief Stop = False, Update _refer_time according to param ref_time.
/// @param ref_time 
void Oscillator::start(unsigned long ref_time)
{
    _stop = false;   
    _ref_time = ref_time;
}

/// @brief Stop = True.
void Oscillator::stop()
{
    _stop = true;
}

/// @brief Alpha = 2 * pi * t / T
/// @param time 
/// @return 
float Oscillator::time_to_radians(double time)
{
    return 2 * PI * time / _period;
}

/// @brief Degree * pi / 180 = alpha
/// @param degrees 
/// @return 
float Oscillator::degrees_to_radians(float degrees)
{
    return degrees * PI / 180;
}

/// @brief Combine time_to_radians and degrees_to_radians: 
/// @param degrees 
/// @return 
float Oscillator::degrees_to_time(float degrees)
{
    return degrees * _period / 360;
}

/// @brief Below are set functions.
/// @param period 
void Oscillator::setPeriod(int period)
{
    _period = period;
}
void Oscillator::setAmplitude(int amplitude)
{
    _amplitude = amplitude;
}
void Oscillator::setPhase(int phase)
{
    _phase = phase;
}
void Oscillator::setOffset(int offset)
{
    _offset = offset;
}
void Oscillator::setTime(unsigned long ref)
{
    _ref_time = ref;
}

/// @brief Return sin output.
/// @return 
float Oscillator::getOutput()
{
    return _output;
}

/// @brief Todo
/// @return 
float Oscillator::getPhaseProgress()
{
    return 0.0;
}

/// @brief Return time.
/// @return 
unsigned long Oscillator::getTime()
{
    return _ref_time;
} */
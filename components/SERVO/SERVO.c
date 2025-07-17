/**
 * @file SERVO.c
 * @author bignut
 * @brief 
 * @version 0.1
 * @date 2025-07-16
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "SERVO.h"
#include "stdio.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


/*
    Using the simplest way to control servo.(pwm output which utilize the esp's ledc controller.)

    Gpio pin is same as the I2C screen's Gpio which is GPIO_22
*/
void servo_pwm_init_ltl()
{
    // pwm0
    ledc_timer_config_t ledc_timer0 = {
        .speed_mode         = LEDC_LOW_SPEED_MODE,                  // speed
        .duty_resolution    = LEDC_TIMER_13_BIT,                    // count resolution
        .timer_num          = LEDC_TIMER_0,                         // timer number
        .freq_hz            = (50),                                 // timer output freq 50Hz equals to 20ms time-period.
        .clk_cfg            = LEDC_AUTO_CLK,                        // auto check clock input
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer0));

    ledc_channel_config_t ledc_channel0 = {
        .speed_mode         = LEDC_LOW_SPEED_MODE,
        .channel            = LEDC_CHANNEL_0,               // ledc channel
        .timer_sel          = LEDC_TIMER_0,
        .intr_type          = LEDC_INTR_DISABLE,            // no interrupy
        .gpio_num           = GPIO_NUM_22,                  // output io
        .duty               = 0,                            // duty = 0
        .hpoint             = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel0));    
}

/// @brief Set pwm0 duty.
/// @param percent 
void servo_set_pwm_0(double percent)
{
    // printf(": %f\n : ", percent);
    assert(percent <= 1.0 && percent >= 0.0);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, percent * 8192));   // 8192 = 2**13 | 13 comes from timer's resolution(13 bits).

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    printf(" set dutty is: %d, get is : %ld\n ", (int)(percent * 8192), ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

/*
    The pwm way to control servo to rotate to fixed angle. Example code.

    
    servo_pwm_init_ltl();

    int init_pwm_n = 25;
    
    while(1)
    {
        vTaskDelay(4000 / portTICK_PERIOD_MS);

        init_pwm_n += 25;

        if(init_pwm_n == 150)
        {
            init_pwm_n = 25;
        }

        servo_set_pwm_0(init_pwm_n * 0.1 * 0.01);

        printf("%d\n", init_pwm_n);

    } 
*/
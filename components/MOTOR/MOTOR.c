/**
 * @file MOTOR.c
 * @author bignut
 * @brief 
 * @details
 *      -> clock -> timer -> channel -> ledc_pwm
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "MOTOR.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// left wheel
#define DRV8833_AIN1_GPIO       GPIO_NUM_22
#define DRV8833_AIN2_GPIO       GPIO_NUM_23

// right wheel
#define DRV8833_BIN1_GPIO       GPIO_NUM_19
#define DRV8833_BIN2_GPIO       GPIO_NUM_21

// universial pwm config
#define PWM_MODE                   LEDC_LOW_SPEED_MODE
#define PWM_DUTY_RES               LEDC_TIMER_13_BIT
#define PWM_FREQUENCY              (4000)


#define LEFT_PWM0_TIMER                 LEDC_TIMER_0    
#define LEFT_PWM0_OUTPUT_IO             DRV8833_AIN1_GPIO
#define LEFT_PWM0_CHANNEL               LEDC_CHANNEL_0


#define LEFT_PWM1_TIMER                 LEDC_TIMER_1 
#define LEFT_PWM1_OUTPUT_IO             DRV8833_AIN2_GPIO
#define LEFT_PWM1_CHANNEL               LEDC_CHANNEL_1


#define RIGHT_PWM2_TIMER                 LEDC_TIMER_2  
#define RIGHT_PWM2_OUTPUT_IO             DRV8833_BIN1_GPIO
#define RIGHT_PWM2_CHANNEL               LEDC_CHANNEL_2


#define RIGHT_PWM3_TIMER                 LEDC_TIMER_3  
#define RIGHT_PWM3_OUTPUT_IO             DRV8833_BIN2_GPIO
#define RIGHT_PWM3_CHANNEL               LEDC_CHANNEL_3

#define PCNT_HIGH_LIMIT                 10000           // 16bit max
#define PCNT_LOW_LIMIT                  -10000

#define PCNT_ENCODER_A_RIGHT                  GPIO_NUM_18   // right_encode_c2
#define PCNT_ENCODER_B_RIGHT                  GPIO_NUM_17   // right_encode_c1

#define PCNT_ENCODER_A_LEFT                  GPIO_NUM_15   // left_encode_c1
#define PCNT_ENCODER_B_LEFT                  GPIO_NUM_16   // left_encode_c2



static const char *TAG = "MORTOR: ";


static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueSetHandle_t)user_ctx;

    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);   // add watch_point_value to queue

    return (high_task_wakeup == pdTRUE);
}


/// @brief Init specified GPIOA and GPIOB as rotate encoder of motor.
/// @param Encode_GPIOA 
/// @param Encode_GPIOB 
/// @return 
static pcnt_unit_handle_t Encoder_unit_init(int Encode_GPIOA, int Encode_GPIOB)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };

    // unit
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));



    // filter
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    /*
        channel begin
    
    */
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = Encode_GPIOA,
        .level_gpio_num = Encode_GPIOB,
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = Encode_GPIOB,
        .level_gpio_num = Encode_GPIOA,   // diff_1
    };

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    // level == high
    //      edge 0 -> 1 : dec
    //      edge 1 -> 0 : inc
    // level == low
    //      edge 0 -> 1 : inc
    //      edge 1 -> 0 : dec
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));


    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE)); // diff_2
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));


    return pcnt_unit;
}


/// @brief Init two encoder of motor in my car.
static void pcnt_init()
{
    pcnt_unit_handle_t pcnt_unit_left = Encoder_unit_init(PCNT_ENCODER_A_LEFT, PCNT_ENCODER_B_LEFT);
    pcnt_unit_handle_t pcnt_unit_right = Encoder_unit_init(PCNT_ENCODER_A_RIGHT, PCNT_ENCODER_B_RIGHT);
    


    int watch_point[] = {PCNT_LOW_LIMIT, -50, 0, 50, PCNT_HIGH_LIMIT};

    for(size_t i=0; i<sizeof(watch_point)/sizeof(watch_point[0]); ++i)
    {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_right, watch_point[i]));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_left, watch_point[i]));
    }
    
    // ISR 
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };

    QueueHandle_t queue_right = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_right, &cbs, queue_right));

    QueueHandle_t queue_left = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit_left, &cbs, queue_left));


    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_right));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_right));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_right));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_left));

    int pulse_count_right = 0;
    int pulse_count_left = 0;
    int event_count = 0;
    

    // used to return MOTOR SPEED.
    while(0)
    {

            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_right, &pulse_count_right));
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_right));
            ESP_LOGI(TAG, "Pulse count right  %d", pulse_count_right);

            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_left, &pulse_count_left));
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_left));
            ESP_LOGI(TAG, "Pulse count left %d", pulse_count_left);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}





/// @brief Init four pwm ouput for drv8833.
static void pwm_init()
{
    // pwm0
    ledc_timer_config_t ledc_timer0 = {
        .speed_mode         = PWM_MODE,                    // speed
        .duty_resolution    = PWM_DUTY_RES,               // count resolution
        .timer_num          = LEFT_PWM0_TIMER,             // timer number
        .freq_hz            = PWM_FREQUENCY,               // timer output freq
        .clk_cfg            = LEDC_AUTO_CLK,               // auto check clock input
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer0));

    ledc_channel_config_t ledc_channel0 = {
        .speed_mode         = PWM_MODE,
        .channel            = LEFT_PWM0_CHANNEL,            // ledc channel
        .timer_sel          = LEFT_PWM0_TIMER,
        .intr_type          = LEDC_INTR_DISABLE,            // no interrupy
        .gpio_num           = LEFT_PWM0_OUTPUT_IO,          // output io
        .duty               = 0,                            // duty = 0
        .hpoint             = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel0));


    // pwm1
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode         = PWM_MODE,                    // speed
        .duty_resolution    = PWM_DUTY_RES,               // count resolution
        .timer_num          = LEFT_PWM1_TIMER,             // timer number
        .freq_hz            = PWM_FREQUENCY,               // timer output freq
        .clk_cfg            = LEDC_AUTO_CLK,               // auto check clock input
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode         = PWM_MODE,
        .channel            = LEFT_PWM1_CHANNEL,            // ledc channel
        .timer_sel          = LEFT_PWM1_TIMER,
        .intr_type          = LEDC_INTR_DISABLE,            // no interrupy
        .gpio_num           = LEFT_PWM1_OUTPUT_IO,          // output io
        .duty               = 0,                            // duty = 0
        .hpoint             = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    // pwm2
    ledc_timer_config_t ledc_timer2 = {
        .speed_mode         = PWM_MODE,                    // speed
        .duty_resolution    = PWM_DUTY_RES,               // count resolution
        .timer_num          = RIGHT_PWM2_TIMER,             // timer number
        .freq_hz            = PWM_FREQUENCY,               // timer output freq
        .clk_cfg            = LEDC_AUTO_CLK,               // auto check clock input
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer2));

    ledc_channel_config_t ledc_channel2 = {
        .speed_mode         = PWM_MODE,
        .channel            = RIGHT_PWM2_CHANNEL,            // ledc channel
        .timer_sel          = RIGHT_PWM2_TIMER,
        .intr_type          = LEDC_INTR_DISABLE,            // no interrupy
        .gpio_num           = RIGHT_PWM2_OUTPUT_IO,          // output io
        .duty               = 0,                            // duty = 0
        .hpoint             = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));
    
    
    // pwm3
    ledc_timer_config_t ledc_timer3 = {
        .speed_mode         = PWM_MODE,                    // speed
        .duty_resolution    = PWM_DUTY_RES,               // count resolution
        .timer_num          = RIGHT_PWM3_TIMER,             // timer number
        .freq_hz            = PWM_FREQUENCY,               // timer output freq
        .clk_cfg            = LEDC_AUTO_CLK,               // auto check clock input
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer3));

    ledc_channel_config_t ledc_channel3 = {
        .speed_mode         = PWM_MODE,
        .channel            = RIGHT_PWM3_CHANNEL,            // ledc channel
        .timer_sel          = RIGHT_PWM3_TIMER,
        .intr_type          = LEDC_INTR_DISABLE,            // no interrupy
        .gpio_num           = RIGHT_PWM3_OUTPUT_IO,          // output io
        .duty               = 0,                            // duty = 0
        .hpoint             = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel3));

}
/// @brief Set pwm0 duty.
/// @param percent 
static void set_pwm_0(double percent)
{
    assert(percent <= 1.0 && percent >= 0.0);

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, LEFT_PWM0_CHANNEL, percent * 8192));   // 8192 = 2**13

    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, LEFT_PWM0_CHANNEL));
}

/// @brief Set pwm1 duty.
/// @param percent 
static void set_pwm_1(double percent)
{
    assert(percent <= 1.0 && percent >= 0.0);

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, LEFT_PWM1_CHANNEL, percent * 8192));   // 8192 = 2**13

    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, LEFT_PWM1_CHANNEL));
}

/// @brief Set pwm2 duty.
/// @param percent 
static void set_pwm_2(double percent)
{
    assert(percent <= 1.0 && percent >= 0.0);

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, RIGHT_PWM2_CHANNEL, percent * 8192));   // 8192 = 2**13

    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, RIGHT_PWM2_CHANNEL));
}

/// @brief Set pwm3 duty.
/// @param percent 
static void set_pwm_3(double percent)
{
    assert(percent <= 1.0 && percent >= 0.0);

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, RIGHT_PWM3_CHANNEL, percent * 8192));   // 8192 = 2**13

    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, RIGHT_PWM3_CHANNEL));
}

static void move_forward()
{   
    double duty = 0.5;
    set_pwm_0(duty);
    set_pwm_1(0);
    set_pwm_3(duty); // 0132 is because two motors has opposite orientation.
    set_pwm_2(0);
}

static void move_back()
{
    double duty = 0.5;
    set_pwm_0(0);
    set_pwm_1(duty);
    set_pwm_3(0);
    set_pwm_2(duty);
}

static void left_control()
{

}

static void right_control()
{

}


void motor_init()
{
    pwm_init();
    pcnt_init();
}

/// @brief Use speed and angle control car's movement.
/// @param speed_mm_s 
/// @param angle_arc 
void motor_control(int speed_mm_s, double angle_arc)
{
    if(speed_mm_s > 0)
    {
        move_forward();
    }
    else
    {
        move_back();
    }
}


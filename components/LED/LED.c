/**
 * @file LED.c
 * @author bignut
 * @brief 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "LED.h"
#include "stdio.h"

void led_init_ltl()
{
    gpio_config_t gpio_init_structure = {0};

    gpio_init_structure.intr_type = GPIO_INTR_DISABLE;
    gpio_init_structure.mode = GPIO_MODE_INPUT_OUTPUT;
    gpio_init_structure.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_init_structure.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_init_structure.pin_bit_mask = 1ull << LED_GPIO_LTL;

    gpio_config(&gpio_init_structure);

    LED(1);
}
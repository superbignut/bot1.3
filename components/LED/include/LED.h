/**
 * @file LED.h
 * @author bignut
 * @brief 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __LTL_LED__
#define __LTL_LED__

#include "driver/gpio.h"


#ifdef __cplusplus
extern "C" {
#endif

#define LED_GPIO_LTL GPIO_NUM_5 /**> led gpio in esp-wroom-32 */

/** @enum
 * 
 * @brief Two GPIO State, on and off
 * 
 */
enum GPIO_OUTPUT_STATE
{
  PIN_LOW = 0,
  PIN_HIGH = 1,
};


/**
 * @brief LED(x) c_define used to set led's state. if x > 0 : high else x < 0 : low
 * 
 */
#define LED(x)  do{x ? gpio_set_level(LED_GPIO_LTL, PIN_HIGH) : gpio_set_level(LED_GPIO_LTL, PIN_LOW);} while(0)

/**
 * @brief LED_SWITCH c_define used to switch led's state according to its current state.
 * 
 */
#define LED_SWITCH() do{gpio_set_level(LED_GPIO_LTL, !gpio_get_level(LED_GPIO_LTL));} while(0)


void led_init_ltl();

#ifdef __cplusplus
}
#endif

#endif
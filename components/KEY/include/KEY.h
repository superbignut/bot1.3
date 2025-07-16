/**
 * @file KEY.h
 * @author bignut
 * @brief 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#ifndef __LTL_KEY__
#define __LTL_KEY__

#include "driver/gpio.h"


#ifdef __cplusplus
extern "C" {
#endif

#define BOOT_GPIO_PIN GPIO_NUM_0    /**> BOOT BOTTOM in esp32-wroom32 */

#define GET_BOOT_STATE  gpio_get_level(BOOT_GPIO_PIN)   /** Get boot-bottom state */

/** @enum
 * @brief Two state of button.
 * 
 */
enum BOOT_BUTTON_STATE
{
    BUTTON_RELEASE,
    BUTTON_PRESS,
};

void key_init_ltl(void);

uint8_t key_scan_ltl(void);

#ifdef __cplusplus
}
#endif

#endif
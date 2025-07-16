/**
 * @file KEY.c
 * @author bignut
 * @brief 
 * @version 0.1
 * @date 2025-03-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "KEY.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "../../LED/include/LED.h"

// static char *logg = "Test";
/**
 * @brief 
 * 
 * @param arg 
 */
static void IRAM_ATTR exit_gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    
    //vTaskDelay(10 / portTICK_PERIOD_MS);

    if(gpio_num == BOOT_GPIO_PIN)
    {
        // printf("%d\n", GET_BOOT_STATE);
        LED_SWITCH();
        // ESP_LOGI(logg, "%d\n", GET_BOOT_STATE);
    }
}

/**
 * @brief Key init function in esp32-wroom-32.( replace Gpio-init to Ex-Interrupt.)
 * 
 */
void key_init_ltl(void){
/*     gpio_config_t gpio_init_structure = {0};

    gpio_init_structure.intr_type = GPIO_INTR_DISABLE;
    gpio_init_structure.mode = GPIO_MODE_INPUT;             
    gpio_init_structure.pull_up_en = GPIO_PULLUP_ENABLE;    
    gpio_init_structure.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_init_structure.pin_bit_mask = 1ull << BOOT_GPIO_PIN;

    gpio_config(&gpio_init_structure); */

    gpio_config_t gpio_init_structure = {0};

    gpio_init_structure.intr_type = GPIO_INTR_NEGEDGE;
    gpio_init_structure.mode = GPIO_MODE_INPUT;             
    gpio_init_structure.pull_up_en = GPIO_PULLUP_ENABLE;    
    gpio_init_structure.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_init_structure.pin_bit_mask = 1ull << BOOT_GPIO_PIN;

    gpio_config(&gpio_init_structure);

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

    gpio_isr_handler_add(BOOT_GPIO_PIN, exit_gpio_isr_handler, (void*)BOOT_GPIO_PIN);
}

/**
 * @brief Check button's state, which should be used in while or thread.
 * 
 * @return uint8_t 
 */
uint8_t key_scan_ltl(void){
    uint8_t key_state = BUTTON_RELEASE;   

    static bool button_is_release = true;
    
    if(button_is_release && (GET_BOOT_STATE == BUTTON_PRESS)) /**> first press */
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);

        if(GET_BOOT_STATE == BUTTON_PRESS)
        {
            button_is_release = false;
            key_state = BUTTON_PRESS;
        }
    }
    else if(GET_BOOT_STATE == BUTTON_RELEASE)   /**> first release */
    {
        button_is_release = true;
    }
    else    /**> still pressing. */
    {
        ;
    }

    return key_state;
}
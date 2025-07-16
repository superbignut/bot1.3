#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "LED.h"
#include "KEY.h"
#include "MOTOR.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "I2Cdev.h"
#include "mpu.h"
#include "esp_log.h"
#include "LD14.h"
#include "WLFL.h"
#include "string.h"
#include "SOCKET.h"
#include "PCA9685.h"

static const char *TAG = "Example";

extern void task_PCA9685(void *ignore);

extern "C" void app_main(void)
{   

    esp_err_t ret;

    ret = nvs_flash_init(); 
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // LD14_lnlt();

    // wlfl_init_sta();
    
    // server_task();

    // pca9685_init();
    
    xTaskCreate(task_PCA9685, "task_PCA9685", 1024 * 2, (void* ) 0, 10, NULL);

    while(1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // get_ip_address_uint16(ip_addr);

        // printf("%d, %d, %d, %d\n", ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);

        // get_ip_address_str(ip_str, 16);

        // printf("%s, %d\n", ip_str, strlen(ip_str));

        printf("1111\n");   

    } 
    
}

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
#include "SERVO.h"
#include "esp_timer.h"
// #include "OSCILLATOR.h"
#include "LROS.h"
#include "XMONKEY.h"
#include <cmath>

static const char *TAG = "Example";

// extern void task_PCA9685(void *ignore);



extern "C" void app_main(void)
{   

    esp_err_t ret;

    ret = nvs_flash_init(); 
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    create_PCA9685_New_Task();

    // MONKEY monkey;
    
    // esp_timer_init(); // 全局初始化，以便后续调用获取 时间函数

    // LD14_lnlt();

    wlfl_init_sta();

    // 启动监听进程
    xTaskCreate(my_tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    // vTaskDelay(2000 / portTICK_PERIOD_MS);

    // LRosInit(); // ros 发布节点初始化
    
    // server_task();

    // pca9685_init();
    
    // xTaskCreate(task_PCA9685, "task_PCA9685", 1024 * 2, (void* ) 0, 10, NULL);

    // task_PCA9685(NULL);

    // float t = 1;

    printf("Compile Successfully!\n");

    while(1)
    {
        // monkey.walk(1000.0, 100);

        // monkey.set_status(X_WALK_F);

        // monkey.main_loop();

        // printf("%.2f\n", sin(++t));
        for(int i=0; i<8; ++i)
        {
            MY_PCA9685_SET_ANGLE(i, 90);
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        for(int i=0; i<8; ++i)
        {
            // MY_PCA9685_SET_ANGLE(i, 60);
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        for(int i=0; i<8; ++i)
        {
            // MY_PCA9685_SET_ANGLE(i, 120);
        }

        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // printf("%.2f, %.2f\n ", asin(0.5), sin(3.1415 / 6));
    }
}

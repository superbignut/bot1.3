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
// #include "XMONKEY.h"
#include "XSNAKE.h"
#include <cmath>

// extern void task_PCA9685(void *ignore);

extern SNAKE global_snake;

extern void adc_main(void);

extern void mesh_main(void);

extern "C" void app_main(void)
{   

    esp_err_t ret;

    ret = nvs_flash_init(); 
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    // wlfl_init_sta();

    // 启动XSNAKE命令监听进程
    // xTaskCreate(my_tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    // 启动pwm电机初始化
    // motor_init();

    // 电机控制
    // motor_control(1, 0);
    
    // 进行 adc读取
    // adc_main(); 

    mesh_main();

    printf("Compile Successfully!\n");

    while(1)
    {               
        vTaskDelay(100 / portTICK_PERIOD_MS);

        printf("%d\n", global_snake.socketCmd);

    }
}

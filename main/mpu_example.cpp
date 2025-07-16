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
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"
#include "I2Cdev.h"
//#include "driver/ledc.h"

// static const char *TAG = "Example";

#define PIN_SDA GPIO_NUM_33         // SCL and SDA GPIO can't be mixed.
#define PIN_SCL GPIO_NUM_32         //

int16_t fifo_buffer[6];

static char TAG[] = "ACC: ";

void tmp_I2C_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_SDA;
    conf.scl_io_num = PIN_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000; // 400 k_bit/s.
    conf.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI("I2C", "I2C initialized successfully");
}

void tmp_mpu(){
    MPU6050 mpu = MPU6050();
    mpu.initialize();
    // mpu.dmpInitialize();

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    // mpu.setDMPEnabled(true);


    while(1)
    {
            mpu.getMotion6(&fifo_buffer[0], &fifo_buffer[1], &fifo_buffer[2],   \
                            &fifo_buffer[3], &fifo_buffer[4], &fifo_buffer[5]);
            
            ESP_LOGI(TAG,"%2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f",    fifo_buffer[0] * 1.0 / 16384,   \
                                                                        fifo_buffer[1] * 1.0 / 16384,   \
                                                                        fifo_buffer[2] * 1.0 / 16384,   \
                                                                        fifo_buffer[3] * 1.0 / 131,     \
                                                                        fifo_buffer[4] * 1.0 / 131 ,    \
                                                                        fifo_buffer[5] * 1.0 / 131); 
            vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
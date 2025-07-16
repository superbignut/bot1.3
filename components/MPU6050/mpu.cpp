#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"
#include "I2Cdev.h"


#define PIN_SDA GPIO_NUM_33         // SCL and SDA GPIO can't be mixed.
#define PIN_SCL GPIO_NUM_32         //

static int16_t mpu_buffer[6];

MPU6050 mpu = MPU6050();

/// @brief Init I2C GPIO, Attention! Some ESP32 pin can't be configed as OUTPUT, which also can't be config as I2C.
static void mpu_I2C_init()
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
    ESP_LOGI("I2C", "I2C initialized successfully...");
}

/// @brief MPU6050 init Using after I2C pin Init.
static void mpu_init(){
    mpu.initialize();
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

}

/// @brief Get acc and w of mpu6050.
/// @param ax 
/// @param ay 
/// @param az 
/// @param wx 
/// @param wy 
/// @param wz 
void mpu_get_aw(float* ax, float* ay, float* az, float* wx, float* wy, float* wz)
{
    mpu.getMotion6(&mpu_buffer[0], &mpu_buffer[1], &mpu_buffer[2], &mpu_buffer[3], &mpu_buffer[4], &mpu_buffer[5]);
    *ax = mpu_buffer[0] * 1.0 / 16384;
    *ay = mpu_buffer[1] * 1.0 / 16384;
    *az = mpu_buffer[2] * 1.0 / 16384;
    *wx = mpu_buffer[3] * 1.0 / 131;
    *wy = mpu_buffer[4] * 1.0 / 131;
    *wz = mpu_buffer[5] * 1.0 / 131;
}

/// @brief MPU6050 init function for superbigcar
void mpu_6050_lnlt()
{
    mpu_I2C_init();
    mpu_init();
}
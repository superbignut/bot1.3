/**
 *  @file pca9685Test.c
 *  
 *  @author     Jonas Scharpf <jonas@brainelectronics.de>
 *  @date       Januar 2018
 *  @version    1.0
 *  
 *  @brief      set PWM of outputs of PCA9685 slave chip
 *  
 *  Description:
 *      I2C Slave device PCA9685 at adress 0x40 can be controlled
 *      to set individual PWM to the outputs
 *      The PWM frequency can only be set for all outputs to same value
 *      
 *  Circuit:
 *      PCA9685 attached to pins 4, 5
 *      I2C Connection:
 *          Board   SDA       SCL
 *          ESP32   any (5)   any (4)
 *          Mega    20        21
 *          Tiny    D0/pin 5  D2/pin 7  (Digispark, Digispark Pro)
 *          Uno     A4        A5
 *  
 */
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "PCA9685.h"

#include "sdkconfig.h"

/**
 * Change according to situation.
 * 
 */
#define I2C_EXAMPLE_MASTER_SCL_IO   GPIO_NUM_32    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO   GPIO_NUM_33    /*!< gpio number for I2C master data  */

/*
    Standard I2C Speed.
*/
#define I2C_EXAMPLE_MASTER_FREQ_HZ  100000      /*!< I2C master clock frequency */

#define I2C_EXAMPLE_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */


#define I2C_ADDRESS     0x40    /*!< lave address for PCA9685 */

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

#define PCA_9685_FREQ (1 / 0.02) // 20ms is required by servo. so freq is set to 50
#define SERVO_PERIOD 20 // period = 20ms
#define PCA_REG_MAX 4096
#define MIN_DUTY_NUM 0.5 // 0.5ms  angle = 0
#define MAX_DUTY_NUM 2.5 // 2.5ms  angle = 180

/// @brief Convert angle (0-180)to relative time (0-20ms) to 9685's 4096's register num.
/// @param angle 
/// @return 
int PCA_9685_Angle_to_Num(int angle)
{
    float ret = ((angle - 0) * 1.0 / 180 * (MAX_DUTY_NUM - MIN_DUTY_NUM) + MIN_DUTY_NUM) / SERVO_PERIOD * PCA_REG_MAX;

    return (int)ret;
}

static char tag[] = "PCA9685";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init(void)
{
    ESP_LOGD(tag, ">> PCA9685");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    if (conf.master.clk_speed < 100000 || conf.master.clk_speed > 1000000) {
        printf("Invalid I2C frequency!\n");
        return;
    }
    
    i2c_port_t i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));  
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0));
}

void task_PCA9685(void *ignore)
{
    printf("Executing on core %d\n", xPortGetCoreID());

    esp_err_t ret;

    i2c_example_master_init();

    set_pca9685_adress(I2C_ADDRESS);

    resetPCA9685();

    setFrequencyPCA9685(PCA_9685_FREQ); // Changed to small one 

    turnAllOff();

    printf("Finished setup, entering loop now\n");
    
    while(1)
    {
        // fade up and down each pin with static logarithmic table
        // see Weber Fechner Law


        
            int num = PCA_9685_Angle_to_Num(90);
            
            for(int angle = 90; angle < 100; ++angle){
                for(int pin = 4; pin < 8; ++pin)
                {
                    setPWM(pin, 0, PCA_9685_Angle_to_Num(angle));   // on
                }
                vTaskDelay(200/ portTICK_PERIOD_MS);
            }
            
            /* for(int angle = 95; angle > 60; --angle){
                //for(int pin = 0; pin < 4; ++pin)
                //{
                setPWM(2, 0, PCA_9685_Angle_to_Num(angle));   // on
                //}
                vTaskDelay(200/ portTICK_PERIOD_MS);
            } */
            

            
        

    }

    vTaskDelete(NULL);
}
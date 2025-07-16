#ifndef _MPU_LTL_H_
#define _MPU_LTL_H_


void mpu_get_aw(float* ax, float* ay, float* az, float* wx, float* wy, float* wz);

void mpu_6050_lnlt();

/*
        mpu_get_aw(&mpu_buffer[0], &mpu_buffer[1], &mpu_buffer[2], &mpu_buffer[3], &mpu_buffer[4], &mpu_buffer[5]);
        ESP_LOGI(TAG,"%2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f",
                    mpu_buffer[0],   \
                    mpu_buffer[1],   \
                    mpu_buffer[2],   \
                    mpu_buffer[3],   \
                    mpu_buffer[4],   \
                    mpu_buffer[5]); 
    } 
*/
#endif
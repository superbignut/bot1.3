idf_component_register(SRCS "MPU6050.cpp" "mpu.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES I2Cdev
)
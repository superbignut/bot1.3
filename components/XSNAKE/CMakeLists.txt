idf_component_register(
    SRCS "XSNAKE.c" 

    INCLUDE_DIRS "include"

    REQUIRES driver esp_timer PCA9685
    
    PRIV_REQUIRES)
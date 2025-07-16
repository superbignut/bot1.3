idf_component_register(
    SRCS "WLFL.c" 

    INCLUDE_DIRS "include"

    REQUIRES driver esp_wifi nvs_flash
    
    PRIV_REQUIRES)
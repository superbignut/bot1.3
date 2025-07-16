idf_component_register(
    SRCS "CLIENT.c" "SERVER.c" 

    INCLUDE_DIRS "include"

    REQUIRES driver esp_wifi nvs_flash WLFL
    
    PRIV_REQUIRES)
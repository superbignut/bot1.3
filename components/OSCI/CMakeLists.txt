idf_component_register(
    SRCS "OSCILLATOR.cpp" 

    INCLUDE_DIRS "include"

    REQUIRES driver esp_timer
    
    PRIV_REQUIRES)
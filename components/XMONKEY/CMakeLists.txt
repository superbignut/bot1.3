idf_component_register(
    SRCS "XMONKEY.cpp" 

    INCLUDE_DIRS "include"

    REQUIRES driver OSCILLATOR PCA9685
    
    PRIV_REQUIRES)
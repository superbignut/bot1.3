idf_component_register(
    SRCS "PCA9685.c" "pca_main.c"

    INCLUDE_DIRS "include"

    REQUIRES driver
    
    PRIV_REQUIRES)
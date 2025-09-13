idf_component_register(
    SRCS "LROS.c" 

    INCLUDE_DIRS "include"

    REQUIRES driver micro_ros_espidf_component WLFL
    
    PRIV_REQUIRES)
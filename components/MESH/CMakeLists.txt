idf_component_register(
    SRCS "MESH.c" 

    INCLUDE_DIRS "include"

    REQUIRES driver esp_wifi
    
    PRIV_REQUIRES)
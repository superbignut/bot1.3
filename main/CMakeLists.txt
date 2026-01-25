idf_component_register(SRCS "mqtt_main.cpp" "mesh_main.cpp" "main.cpp"  # Attention! main.c is not "main.c"
                                        # "mpu_example.cpp"
                            "motor_adc.cpp"
                            "mesh_main.cpp"
                            "mqtt_main.cpp"
                         INCLUDE_DIRS 
                         PRIV_REQUIRES)

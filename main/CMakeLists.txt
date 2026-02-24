idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES relay_manager lora_manager sd_manager modbus_manager network_manager)  
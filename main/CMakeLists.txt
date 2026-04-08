idf_component_register( 
    SRCS "main.c" 
    INCLUDE_DIRS "." 
    REQUIRES common lora_manager sd_manager network_manager relay_manager nvs_flash driver efuse
)
idf_component_register(
    SRCS "network_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES common esp_wifi mqtt nvs_flash esp_event
)

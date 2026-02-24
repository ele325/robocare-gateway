idf_component_register(SRCS "lora_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver)  # Utilisez "driver" au lieu de "spi"
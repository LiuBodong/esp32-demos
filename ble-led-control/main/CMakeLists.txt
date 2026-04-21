set(SRCS "main.c")

idf_component_register(SRCS ${SRCS}
                       PRIV_REQUIRES bt esp_driver_gpio esp_driver_ledc nvs_flash
                       INCLUDE_DIRS ".")

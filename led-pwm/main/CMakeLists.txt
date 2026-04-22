set(SRCS "main.c")

idf_component_register(SRCS ${SRCS}
                       PRIV_REQUIRES esp_driver_ledc
                       INCLUDE_DIRS ".")

set(SRCS "main.c")

idf_component_register(SRCS ${SRCS}
                       PRIV_REQUIRES lvgl esp_lvgl_port esp_lcd
                       INCLUDE_DIRS ".")

set(SRCS "main.c")

idf_component_register(SRCS ${SRCS}
                       PRIV_REQUIRES lvgl esp_lvgl_port esp_lcd
                                     esp_wifi esp_netif nvs_flash
                       INCLUDE_DIRS ".")

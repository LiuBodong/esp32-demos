set(SRCS "main.c"
         "wifi_manager.c"
         "ntp_sync.c"
         "oled_driver.c"
         "clock_ui.c")

idf_component_register(SRCS ${SRCS}
                       PRIV_REQUIRES lvgl esp_lvgl_port esp_lcd
                                     esp_wifi esp_netif nvs_flash esp_event
                       INCLUDE_DIRS ".")

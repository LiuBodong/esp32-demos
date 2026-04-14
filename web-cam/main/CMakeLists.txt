set(SRCS "main.c" "app_wifi.c" "app_camera.c" "app_httpd.c")

idf_component_register(SRCS ${SRCS}
                       PRIV_REQUIRES esp_wifi esp32-camera esp_http_server nvs_flash 
                       INCLUDE_DIRS ".")

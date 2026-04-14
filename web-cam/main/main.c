#include "app_wifi.h"
#include "app_camera.h"
#include "app_httpd.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

static const char *TAG = "app_webcam";

esp_err_t app_main(void)
{
    // 初始化 WIFI
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase()); // 如果NVS分区有误，先擦除
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = app_wifi_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error init wifi, %s", esp_err_to_name(ret));
        return ret;
    }

    ret = app_camera_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error init camera, %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_handle_t httpd_server = app_httpd_init();
    if (httpd_server)
    {
        ESP_LOGI(TAG, "Successfully initialized!");
        char ip[16] = {0};
        if (app_wifi_get_ip(ip, sizeof(ip)) == ESP_OK)
        {
            ESP_LOGI(TAG, "Please Visite: http://%s:%d", ip, CONFIG_APP_WEBCAM_HTTPD_PORT);
        }
    }
    return ESP_OK;
}
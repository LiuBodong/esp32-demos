/**
 * @file wifi_manager.c
 * @brief WiFi连接管理模块实现
 *
 * 处理WiFi初始化、连接和事件回调
 */

#include "wifi_manager.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

static const char *TAG = "wifi_manager";

/** @brief WiFi连接最大重试次数 */
#define WIFI_MAX_RETRY 10

/** @brief WiFi已成功获取IP地址事件位 */
#define WIFI_CONNECTED_BIT BIT0

/** @brief WiFi连接失败事件位 */
#define WIFI_FAIL_BIT BIT1

/** @brief WiFi SSID（网络名称） */
#define WIFI_SSID CONFIG_APP_DESKTOP_GADGET_WIFI_SSID

/** @brief WiFi密码 */
#define WIFI_PASSWORD CONFIG_APP_DESKTOP_GADGET_WIFI_PASSWORD

/** @brief FreeRTOS事件组句柄 */
static EventGroupHandle_t s_wifi_event_group;

/** @brief WiFi连接重试计数器 */
static int s_retry_num = 0;

/**
 * @brief WiFi和IP事件统一处理回调
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "WiFi STA启动，开始连接...");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_MAX_RETRY)
        {
            ESP_LOGW(TAG, "WiFi连接断开，正在重试... (%d/%d)",
                     s_retry_num + 1, WIFI_MAX_RETRY);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_wifi_connect();
            s_retry_num++;
        }
        else
        {
            ESP_LOGE(TAG, "WiFi连接失败，已达最大重试次数");
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi已连接，IP地址: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_manager_init_and_connect(void)
{
    /* 初始化NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS分区格式不兼容，正在擦除...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 创建事件组 */
    s_wifi_event_group = xEventGroupCreate();

    /* 初始化TCP/IP网络栈 */
    ESP_ERROR_CHECK(esp_netif_init());

    /* 创建默认事件循环 */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 创建默认WiFi STA网络接口 */
    esp_netif_create_default_wifi_sta();

    /* 初始化WiFi驱动 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* 注册事件处理回调 */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    /* 配置WiFi连接参数 */
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };

    /* 设置WiFi模式并启动 */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(8));
    ESP_LOGI(TAG, "已降低WiFi发射功率");

    ESP_LOGI(TAG, "WiFi初始化完成，SSID: %s, PASSWORD: %s", WIFI_SSID, WIFI_PASSWORD);

    /* 等待连接结果 */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "WiFi连接成功");
        return ESP_OK;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "WiFi连接失败");
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGE(TAG, "WiFi连接出现意外状态");
        return ESP_FAIL;
    }
}

EventGroupHandle_t wifi_manager_get_event_group(void)
{
    return s_wifi_event_group;
}
#include "app_wifi.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include <string.h>

// ---------- 事件组标志 ----------
static const int WIFI_CONNECTED_BIT = BIT0; // Wi-Fi 已连接（L2 层）
static const int WIFI_GOT_IP_BIT = BIT1;    // 已获取 IP 地址
static const int WIFI_FAIL_BIT = BIT2;      // 连接失败

static const char *TAG = "app_wifi";
static EventGroupHandle_t s_wifi_event_group = NULL;
static esp_netif_t *s_sta_netif = NULL; // 保存 station 网络接口句柄

// ---------- 静态 IP 配置函数 ----------
static esp_err_t set_static_ip(esp_netif_t *netif)
{
    esp_netif_dhcpc_stop(netif); // 必须先停止 DHCP 客户端

    esp_netif_ip_info_t ip_info;
    memset(&ip_info, 0, sizeof(ip_info));

    // 转换 IP、网关、掩码
    if (!inet_pton(AF_INET, CONFIG_APP_WEBCAM_WIFI_STATIC_IP, &ip_info.ip))
    {
        ESP_LOGE(TAG, "Invalid static IP: %s", CONFIG_APP_WEBCAM_WIFI_STATIC_IP);
        return ESP_ERR_INVALID_ARG;
    }
    if (!inet_pton(AF_INET, CONFIG_APP_WEBCAM_STATIC_GATEWAY, &ip_info.gw))
    {
        ESP_LOGE(TAG, "Invalid gateway: %s", CONFIG_APP_WEBCAM_STATIC_GATEWAY);
        return ESP_ERR_INVALID_ARG;
    }
    if (!inet_pton(AF_INET, CONFIG_APP_WEBCAM_STATIC_NETMASK, &ip_info.netmask))
    {
        ESP_LOGE(TAG, "Invalid netmask: %s", CONFIG_APP_WEBCAM_STATIC_NETMASK);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_netif_set_ip_info(netif, &ip_info);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set static IP: %s", esp_err_to_name(err));
        return err;
    }

    // 设置 DNS
    esp_netif_dns_info_t dns_info;
    if (!inet_pton(AF_INET, CONFIG_APP_WEBCAM_STATIC_DNS, &dns_info.ip.u_addr.ip4))
    {
        ESP_LOGE(TAG, "Invalid DNS: %s", CONFIG_APP_WEBCAM_STATIC_DNS);
        return ESP_ERR_INVALID_ARG;
    }
    dns_info.ip.type = IPADDR_TYPE_V4;
    err = esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set DNS: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Static IP configured: %s", CONFIG_APP_WEBCAM_WIFI_STATIC_IP);
    return ESP_OK;
}

// ---------- 事件处理函数 ----------
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        // Wi-Fi 驱动已启动，尝试连接路由器
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi started, connecting to AP...");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected from AP, reason: %d", disconn->reason);
        // 重连（简单策略：延迟后重连，避免频繁）
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        // 设置事件组标志
        xEventGroupSetBits(s_wifi_event_group, WIFI_GOT_IP_BIT);
    }
}

// ---------- 公共 API ----------
esp_err_t app_wifi_init(void)
{
    // 1. 创建事件组
    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group)
    {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    // 2. 初始化 TCP/IP 协议栈和事件循环
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 3. 创建默认 Wi-Fi Station 网络接口
    s_sta_netif = esp_netif_create_default_wifi_sta();
    assert(s_sta_netif);

    // 4. 根据配置决定是否使用静态 IP
    if (CONFIG_APP_WEBCAM_WIFI_USE_STATIC_IP)
    {
        ESP_LOGI(TAG, "Using static IP configuration");
        esp_err_t err = set_static_ip(s_sta_netif);
        if (err != ESP_OK)
        {
            return err;
        }
    }
    else
    {
        ESP_LOGI(TAG, "Using DHCP (automatic IP)");
        // DHCP 默认是开启的，只需确保它处于运行状态
        esp_netif_dhcpc_start(s_sta_netif);
    }

    // 5. 初始化 Wi-Fi 驱动
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 6. 注册事件处理函数
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    // 7. 配置 Wi-Fi 参数（SSID / 密码）
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_APP_WEBCAM_WIFI_SSID,
            .password = CONFIG_APP_WEBCAM_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // 可根据实际情况调整
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // 关闭省电模式

    // 8. 等待 Wi-Fi 连接成功并获取 IP（超时 15 秒）
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_GOT_IP_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(15000));

    if (bits & WIFI_GOT_IP_BIT)
    {
        ESP_LOGI(TAG, "Wi-Fi connected and IP obtained successfully");
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to get IP address within timeout");
        return ESP_FAIL;
    }
}

// 获取当前 IP 地址（字符串形式）
esp_err_t app_wifi_get_ip(char *ip_str, size_t len)
{
    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(s_sta_netif, &ip_info);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get ip: %s", esp_err_to_name(ret));
        return ret;
    }
    inet_ntop(AF_INET, &ip_info.ip.addr, ip_str, len);
    ESP_LOGI(TAG, "Got IP: %s", ip_str);
    return ESP_OK;
}
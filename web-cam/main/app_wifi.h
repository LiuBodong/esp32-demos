#ifndef APP_WIFI_H
#define APP_WIFI_H

#include "esp_err.h"

/**
 * @brief 初始化WiFi，连接到配置的AP，并设置静态IP（如果配置了）。
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t app_wifi_init(void);

/**
 * @brief 获取当前 Wi-Fi 的 IP 地址字符串
 * @param ip_str 输出缓冲区
 * @param len    缓冲区长度 (建议至少 16)
 * @return ESP_OK 成功，ESP_ERR_INVALID_STATE 未连接或未获取 IP
 */
esp_err_t app_wifi_get_ip(char *ip_str, size_t len);

#endif
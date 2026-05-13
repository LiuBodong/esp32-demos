/**
 * @file wifi_manager.h
 * @brief WiFi连接管理模块
 *
 * 提供WiFi初始化、连接和事件处理功能
 */

#pragma once

#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化WiFi并连接到配置的网络
 *
 * 该函数会阻塞当前任务，直到WiFi连接成功或失败。
 * 连接参数通过menuconfig中的CONFIG_APP_DESKTOP_GADGET_WIFI_SSID和
 * CONFIG_APP_DESKTOP_GADGET_WIFI_PASSWORD配置。
 *
 * @return
 *     - ESP_OK: WiFi连接成功
 *     - ESP_FAIL: WiFi连接失败（超过最大重试次数）
 */
esp_err_t wifi_manager_init_and_connect(void);

/**
 * @brief 获取WiFi事件组句柄
 *
 * 用于其他模块等待WiFi连接状态
 *
 * @return EventGroupHandle_t 事件组句柄
 */
EventGroupHandle_t wifi_manager_get_event_group(void);

#ifdef __cplusplus
}
#endif
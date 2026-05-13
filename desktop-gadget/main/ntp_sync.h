/**
 * @file ntp_sync.h
 * @brief NTP时间同步模块
 *
 * 提供NTP初始化和时间同步功能
 */

#pragma once

#include "esp_err.h"
#include <freertos/FreeRTOS.h>
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化SNTP并同步网络时间
 *
 * 该函数会阻塞当前任务，直到NTP时间同步完成。
 * NTP服务器地址通过menuconfig配置。
 *
 * @param wifi_event_group WiFi事件组句柄，用于等待NTP同步完成事件
 *
 * @return
 *     - ESP_OK: NTP时间同步成功
 *     - ESP_FAIL: NTP时间同步超时（60秒）
 */
esp_err_t ntp_sync_init_and_sync(EventGroupHandle_t wifi_event_group);

#ifdef __cplusplus
}
#endif
/**
 * @file ntp_sync.c
 * @brief NTP时间同步模块实现
 *
 * 处理NTP初始化和时间同步
 */

#include "ntp_sync.h"

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_sntp.h"

#include "sdkconfig.h"

static const char *TAG = "ntp_sync";

/** @brief NTP时间同步完成事件位 */
#define NTP_SYNCED_BIT BIT2

/** @brief 事件组句柄（由init函数设置） */
static EventGroupHandle_t s_event_group = NULL;

/** @brief NTP同步回调函数 */
static void ntp_sync_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP时间同步完成! 时间: %ld", (long)tv->tv_sec);
    if (s_event_group)
    {
        xEventGroupSetBits(s_event_group, NTP_SYNCED_BIT);
    }
}

esp_err_t ntp_sync_init_and_sync(EventGroupHandle_t wifi_event_group)
{
    ESP_LOGI(TAG, "正在初始化NTP时间同步...");

    /* 保存事件组句柄供回调使用 */
    s_event_group = wifi_event_group;

    /* 设置时区 */
    char tz_str[16];
    int gmt_offset = CONFIG_APP_DESKTOP_GADGET_NTP_GMT_OFFSET;
    int dst_offset = CONFIG_APP_DESKTOP_GADGET_NTP_DAYLIGHT_OFFSET;

    if (gmt_offset >= 0)
    {
        snprintf(tz_str, sizeof(tz_str), "CST%d", -gmt_offset / 3600);
    }
    else
    {
        snprintf(tz_str, sizeof(tz_str), "EST%d", -gmt_offset / 3600);
    }

    if (dst_offset != 0)
    {
        char dst_str[32];
        snprintf(dst_str, sizeof(dst_str), "%s,M3.2.0/02:00,M11.1.0/02:00", tz_str);
        setenv("TZ", dst_str, 1);
    }
    else
    {
        setenv("TZ", tz_str, 1);
    }
    tzset();

    ESP_LOGI(TAG, "时区设置: %s (GMT%+d, DST: %d秒)",
             tz_str, gmt_offset / 3600, dst_offset);

    /* 配置SNTP */
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, CONFIG_APP_DESKTOP_GADGET_NTP_SERVER1);
    esp_sntp_setservername(1, CONFIG_APP_DESKTOP_GADGET_NTP_SERVER2);
    esp_sntp_setservername(2, CONFIG_APP_DESKTOP_GADGET_NTP_SERVER3);
    esp_sntp_set_time_sync_notification_cb(ntp_sync_cb);
    esp_sntp_set_sync_interval(3600000);

    /* 启动SNTP服务 */
    esp_sntp_init();

    ESP_LOGI(TAG, "SNTP已启动，正在等待时间同步...");
    ESP_LOGI(TAG, "NTP服务器: %s, %s, %s",
             CONFIG_APP_DESKTOP_GADGET_NTP_SERVER1,
             CONFIG_APP_DESKTOP_GADGET_NTP_SERVER2,
             CONFIG_APP_DESKTOP_GADGET_NTP_SERVER3);

    /* 等待NTP同步完成 */
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        NTP_SYNCED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(60000));

    if (bits & NTP_SYNCED_BIT)
    {
        time_t now;
        struct tm time_info;
        time(&now);
        localtime_r(&now, &time_info);

        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &time_info);
        ESP_LOGI(TAG, "当前时间: %s", time_buf);

        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "NTP时间同步超时（60秒）");
        return ESP_FAIL;
    }
}
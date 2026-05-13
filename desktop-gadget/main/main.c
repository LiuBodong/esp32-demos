/**
 * @file main.c
 * @brief 桌面摆件主程序 - 网络授时时钟
 *
 * 功能概述：
 *   1. 初始化I2C总线，驱动SSD1315 OLED显示屏（128x64）
 *   2. 通过LVGL图形库在OLED上绘制时钟界面
 *   3. 连接WiFi网络，通过NTP服务器获取精确时间
 *   4. 每秒刷新屏幕，显示当前日期和时间
 *
 * 硬件平台：ESP32-C3 + SSD1315 OLED (I2C, 128x64)
 * 软件框架：ESP-IDF 6.1.0 + LVGL 9.5.0 + esp_lvgl_port 2.7.2
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

/* 模块头文件 */
#include "wifi_manager.h"
#include "ntp_sync.h"
#include "oled_driver.h"
#include "clock_ui.h"

static const char *TAG = "desktop-gadget";

/**
 * @brief ESP-IDF应用程序入口点
 *
 * 程序启动后的主执行流程，所有初始化在此函数中顺序完成。
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  桌面摆件 - 网络授时时钟");
    ESP_LOGI(TAG, "  硬件: ESP32-C3 + SSD1315 OLED");
    ESP_LOGI(TAG, "========================================");

    /* 第1步：初始化OLED显示屏和LVGL */
    ESP_LOGI(TAG, "[1/3] 初始化OLED显示屏...");
    esp_err_t oled_ret = oled_driver_init();
    if (oled_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "OLED初始化失败");
        return;
    }

    /* 第2步：连接WiFi网络 */
    ESP_LOGI(TAG, "[2/3] 连接WiFi网络...");
    esp_err_t wifi_ret = wifi_manager_init_and_connect();
    if (wifi_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi连接失败，无法进行NTP同步");

        lvgl_port_lock(0);
        lv_obj_t *scr = lv_screen_active();
        lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);

        lv_obj_t *err_label = lv_label_create(scr);
        lv_label_set_text(err_label, "WiFi Failed!");
        lv_obj_set_style_text_color(err_label, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(err_label, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_align(err_label, LV_ALIGN_CENTER, 0, 0);
        lvgl_port_unlock();

        return;
    }

    /* 第3步：通过NTP同步网络时间 */
    ESP_LOGI(TAG, "[3/3] 同步NTP时间...");
    esp_err_t ntp_ret = ntp_sync_init_and_sync(wifi_manager_get_event_group());
    if (ntp_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "NTP同步失败，将使用本地时间");
    }

    /* 第4步：创建时钟显示界面 */
    ESP_LOGI(TAG, "创建时钟界面...");
    lvgl_port_lock(0);
    clock_ui_create();
    lvgl_port_unlock();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  桌面摆件启动完成！");
    ESP_LOGI(TAG, "========================================");
}
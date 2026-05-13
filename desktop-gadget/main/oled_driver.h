/**
 * @file oled_driver.h
 * @brief OLED屏幕驱动模块
 *
 * 提供SSD1315 OLED屏幕初始化和LVGL配置功能
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化SSD1315 OLED显示屏和LVGL图形库
 *
 * 完成后，LVGL将拥有一个128x64的单色显示设备，
 * 可以通过lvgl_port_lock()/lvgl_port_unlock()线程安全地操作。
 *
 * @return
 *     - ESP_OK: 初始化成功
 *     - ESP_ERR_*: 初始化失败
 */
esp_err_t oled_driver_init(void);

#ifdef __cplusplus
}
#endif
#ifndef APP_CAMERA_H
#define APP_CAMERA_H

#include "esp_err.h"

/**
 * @brief 根据menuconfig的配置初始化摄像头。
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t app_camera_init(void);

#endif
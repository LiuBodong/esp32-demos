#ifndef APP_HTTPD_H
#define APP_HTTPD_H

#include "esp_err.h"
#include "esp_http_server.h"

/**
 * @brief 根据menuconfig的配置初始化Web服务。
 *
 * @return httpd_handle_t
 */
httpd_handle_t app_httpd_init(void);

#endif
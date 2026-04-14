#include "app_httpd.h"

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "httpd_server";

static int get_delay_ms()
{
    int max_fps = CONFIG_APP_WEBCAM_CAMERA_MAX_FPS;
    return (int)(1000 / max_fps);
}

/* ===================== HTML 页面 ===================== */
static const char index_html[] =
    "<!DOCTYPE html>\n"
    "<html>\n"
    "<head>\n"
    "  <meta charset=\"utf-8\">\n"
    "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
    "  <title>ESP32 Camera</title>\n"
    "  <style>\n"
    "    * { margin: 0; padding: 0; box-sizing: border-box; }\n"
    "    body {\n"
    "      background: #111;\n"
    "      display: flex;\n"
    "      flex-direction: column;\n"
    "      align-items: center;\n"
    "      justify-content: center;\n"
    "      min-height: 100vh;\n"
    "      font-family: -apple-system, sans-serif;\n"
    "      color: #ccc;\n"
    "    }\n"
    "    h1 { font-size: 1.2rem; margin-bottom: 16px; color: #e8c547; }\n"
    "    img {\n"
    "      max-width: 95vw;\n"
    "      max-height: 80vh;\n"
    "      border: 2px solid #333;\n"
    "      border-radius: 8px;\n"
    "      background: #000;\n"
    "    }\n"
    "    .status { margin-top: 12px; font-size: 0.8rem; color: #666; }\n"
    "  </style>\n"
    "</head>\n"
    "<body>\n"
    "  <h1>ESP32-CAM Live</h1>\n"
    "  <img src=\"/stream\" alt=\"camera stream\">\n"
    "  <div class=\"status\">MJPEG Stream</div>\n"
    "</body>\n"
    "</html>\n";

/* ===================== 处理器：主页 ===================== */
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* ===================== 处理器：MJPEG 流 ===================== */
static esp_err_t stream_handler(httpd_req_t *req)
{
    int delay_ms = get_delay_ms();
    esp_err_t res = ESP_OK;
    char part_buf[64];

    /* MJPEG 多部分响应头 */
    static const char *stream_content_type =
        "multipart/x-mixed-replace;boundary=frame";

    httpd_resp_set_type(req, stream_content_type);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        /* 构造 MJPEG 分隔符 + 头部 */
        int hlen = snprintf(part_buf, sizeof(part_buf),
                            "--frame\r\n"
                            "Content-Type: image/jpeg\r\n"
                            "Content-Length: %u\r\n\r\n",
                            fb->len);

        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK)
        {
            esp_camera_fb_return(fb);
            break;
        }

        /* 发送 JPEG 数据 */
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (res != ESP_OK)
        {
            break;
        }

        /* 帧尾 */
        res = httpd_resp_send_chunk(req, "\r\n", 2);
        if (res != ESP_OK)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    return res;
}

/* ===================== 处理器：单帧抓拍 ===================== */
static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        ESP_LOGE(TAG, "Capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return ESP_OK;
}

httpd_handle_t app_httpd_init(void)
{
    httpd_handle_t server = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_APP_WEBCAM_HTTPD_PORT;
    config.max_uri_handlers = 4;
    config.stack_size = 8192;       /* 流任务需要较大栈 */
    config.lru_purge_enable = true; /* 连接满时释放旧连接 */

    esp_err_t ret;
    if ((ret = httpd_start(&server, &config)) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    /* 注册 URI */
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &stream_uri);

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &capture_uri);

    ESP_LOGI(TAG, "Camera server ready");
    return server;
}
#include "app_camera.h"
#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "app_camera";

// 根据所选的硬件模块定义摄像头引脚
#if defined(CONFIG_APP_WEBCAM_BOARD_TYPE_ESP32_CAM)

// ================== 经典 ESP32-CAM (AI-Thinker) 专属引脚配置 ==================
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 // 软件复位
#define CAM_PIN_XCLK 0

// I2C 控制总线
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

// 数据线 (Y9~Y2 对应 D7~D0)
#define CAM_PIN_D7 35 // Y9
#define CAM_PIN_D6 34 // Y8
#define CAM_PIN_D5 39 // Y7
#define CAM_PIN_D4 36 // Y6
#define CAM_PIN_D3 21 // Y5
#define CAM_PIN_D2 19 // Y4
#define CAM_PIN_D1 18 // Y3
#define CAM_PIN_D0 5  // Y2

// 同步信号线
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#else

// ESP32-S3-CAM (官方/常见) 引脚定义
// 不同制造商可能有细微差别，此配置基于常见定义。可根据实际情况调整。
// ================== Freenove ESP32-S3 WROOM 专属引脚配置 ==================
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 15 // CAM_XCLK

// I2C 控制总线
#define CAM_PIN_SIOD 4  // CAM_SIOD
#define CAM_PIN_SIOC 5  // CAM_SIOC

// 数据线 (Y9~Y2 对应 D7~D0)
#define CAM_PIN_D7 16   // CAM_Y9
#define CAM_PIN_D6 17   // CAM_Y8
#define CAM_PIN_D5 18   // CAM_Y7
#define CAM_PIN_D4 12   // CAM_Y6
#define CAM_PIN_D3 10   // CAM_Y5
#define CAM_PIN_D2 8    // CAM_Y4
#define CAM_PIN_D1 9    // CAM_Y3
#define CAM_PIN_D0 11   // CAM_Y2

// 同步信号线
#define CAM_PIN_VSYNC 6 // CAM_VSYNC
#define CAM_PIN_HREF 7  // CAM_HREF
#define CAM_PIN_PCLK 13 // CAM_PCLK

#endif

// ======================== 分辨率映射 ========================
static framesize_t get_frame_size(void)
{
#if defined(CONFIG_APP_WEBCAM_FRAMESIZE_QVGA)
    return FRAMESIZE_QVGA; // 320x240
#elif defined(CONFIG_APP_WEBCAM_FRAMESIZE_VGA)
    return FRAMESIZE_VGA; // 640x480
#elif defined(CONFIG_APP_WEBCAM_FRAMESIZE_HD)
    return FRAMESIZE_HD; // 1280x720
#elif defined(CONFIG_APP_WEBCAM_FRAMESIZE_UXGA)
    return FRAMESIZE_UXGA; // 1600x1200
#elif defined(CONFIG_APP_WEBCAM_FRAMESIZE_QXGA)
    return FRAMESIZE_QXGA; // 2048x1536
#elif defined(CONFIG_APP_WEBCAM_FRAMESIZE_QHD)
    return FRAMESIZE_QHD; // 2560x1440 (仅 OV5640)
#elif defined(CONFIG_APP_WEBCAM_FRAMESIZE_5MP)
    return FRAMESIZE_5MP; // 2592x1944 (仅 OV5640)
#else
    return FRAMESIZE_VGA;
#endif
}

// ======================== 帧缓冲数量 ========================
static uint8_t get_fb_count(void)
{
#if defined(CONFIG_APP_WEBCAM_BOARD_TYPE_ESP32_CAM)
    // ESP32 内存紧张，高分辨率只用 1 个缓冲
    framesize_t fs = get_frame_size();
    return (fs >= FRAMESIZE_UXGA) ? 1 : 2;
#else
    // ESP32-S3 有 PSRAM，优先双缓冲避免丢帧
    return 2;
#endif
}

// ======================== 传感器差异化配置 ========================
static void configure_sensor(sensor_t *s)
{
    if (!s)
        return;

    /* ---- 公共基础配置 ---- */
    s->set_brightness(s, 0); // -2 ~ 2
    s->set_contrast(s, 0);   // -2 ~ 2
    s->set_whitebal(s, 1);   // AWB 开启
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);       // 0=Auto
    s->set_exposure_ctrl(s, 1); // AEC 开启
    s->set_gain_ctrl(s, 1);     // AGC 开启
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);

    /* ---- 传感器专属优化 ---- */
    switch (s->id.PID)
    {

    /* ====== OV2640 (2MP, 常见于 ESP32-CAM) ====== */
    case OV2640_PID:
        ESP_LOGI(TAG, "Sensor: OV2640");
        // OV2640 的 AGC 增益范围小，适当放宽天花板
        s->set_gainceiling(s, (gainceiling_t)6); // 0~6, 最大增益
        s->set_agc_gain(s, 0);
        // OV2640 色彩偏暖，稍微降饱和度补偿
        s->set_saturation(s, -1);
        // 低光环境自动曝光补偿
        s->set_aec2(s, 1);        // DSP AEC 开启
        s->set_aec_value(s, 300); // 手动曝光值 (AEC 关闭时生效)
        // 低分辨率下开启降噪/锐化
        s->set_raw_gma(s, 1);        // Gamma 校正
        s->set_lenc(s, 1);           // 镜头校正
        s->set_special_effect(s, 0); // 无特效
        break;

    /* ====== OV3660 (3MP) ====== */
    case OV3660_PID:
        ESP_LOGI(TAG, "Sensor: OV3660");
        // OV3660 有更好的动态范围，AGC 天花板适中
        s->set_gainceiling(s, (gainceiling_t)4);
        s->set_agc_gain(s, 5);
        // OV3660 默认白平衡偏青，手动校正
        s->set_wb_mode(s, 0); // Auto 模式
        s->set_awb_gain(s, 1);
        // 曝光控制更精细
        s->set_aec2(s, 1);
        s->set_aec_value(s, 400);
        // 色彩微调
        s->set_brightness(s, 1);
        s->set_contrast(s, 1);
        // OV3660 支持 BPC (坏点校正)
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);

        // 上下反转
        s->set_vflip(s, 1);
        break;

    /* ====== OV5640 (5MP, 支持 2592x1944) ====== */
    case OV5640_PID:
        ESP_LOGI(TAG, "Sensor: OV5640");
        // OV5640 动态范围最大，AGC 天花板可以低一些
        s->set_gainceiling(s, (gainceiling_t)3);
        s->set_agc_gain(s, 0);
        // 高分辨率下曝光策略：避免过曝
        s->set_aec2(s, 1);
        s->set_aec_value(s, 200);
        // OV5640 色彩还原好，保持默认
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        // 高分辨率下镜头校正和 Gamma 校正尤为重要
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);
        // 高分辨率下帧率受限，开启降噪
        s->set_denoise(s, 1);
        // OV5640 支持 ISP 缩放，大分辨率下很有用
        s->set_quality(s, CONFIG_APP_WEBCAM_CAMERA_QUALITY);
        break;

    default:
        ESP_LOGW(TAG, "Unknown sensor PID: 0x%04X, using defaults", s->id.PID);
        s->set_gainceiling(s, (gainceiling_t)4);
        s->set_aec2(s, 1);
        s->set_raw_gma(s, 1);
        s->set_lenc(s, 1);
        break;
    }

    /* ---- 高分辨率场景的公共调优 ---- */
    framesize_t fs = get_frame_size();
    if (fs >= FRAMESIZE_UXGA)
    {
        // 大分辨率：优先画质，帧率会自然降低
        s->set_quality(s, CONFIG_APP_WEBCAM_CAMERA_QUALITY);
        // 可选：强制降噪
        s->set_denoise(s, 1);
    }
    else
    {
        // 小分辨率：优先帧率，适当放宽画质
        s->set_quality(s, CONFIG_APP_WEBCAM_CAMERA_QUALITY);
        s->set_denoise(s, 0); // 小分辨率关闭降噪以提速
    }

    /* ---- 低光增强（所有传感器通用） ---- */
    // 如果检测到环境较暗，可动态调用以下组合:
    // s->set_agc_gain(s, 15);        // 拉高 AGC
    // s->set_gainceiling(s, (gainceiling_t)6);  // 解锁增益天花板
    // s->set_aec2(s, 1);             // 确保 DSP AEC 开启
}

esp_err_t app_camera_init(void)
{
    framesize_t frame_size = get_frame_size();
    uint8_t fb_count = get_fb_count();

#if defined(CONFIG_APP_WEBCAM_CAMERA_SENSOR_OV3660)
    if (frame_size > FRAMESIZE_QXGA)
    {
        ESP_LOGW(TAG, "OV3660 does not support > QXGA, falling back to QXGA");
        frame_size = FRAMESIZE_QXGA;
    }
#endif
    // OV5640 支持所有分辨率，无需降级

    /* ---- 前置检查：ESP32 内存 vs 分辨率 ---- */
#if defined(CONFIG_APP_WEBCAM_BOARD_TYPE_ESP32_CAM)
    if (frame_size >= FRAMESIZE_UXGA)
    {
        ESP_LOGW(TAG, "ESP32 + UXGA or higher may be unstable, "
                      "consider ESP32-S3 for large resolutions");
    }
#endif

    /* ---- 动态调整 Grab Mode ---- */
    camera_grab_mode_t grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    if (fb_count > 1)
    {
        // 只有双缓冲及以上，才允许开启获取最新帧模式，降低延迟
#if defined(CONFIG_APP_WEBCAM_CAMERA_SENSOR_OV3660)
        grab_mode = CAMERA_GRAB_WHEN_EMPTY; // OV3660 推荐保持顺序获取
#else
        grab_mode = CAMERA_GRAB_LATEST;
#endif
    }

    uint xclk_freq_hz = CONFIG_APP_WEBCAM_CAMERA_XCLK_FREQ_MHZ * 1000 * 1000;

    camera_config_t config = {
        /* 引脚 */
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        /* 时钟 & 通道 */
        .xclk_freq_hz = xclk_freq_hz,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        /* 格式 & 分辨率 */
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = frame_size,
        .jpeg_quality = CONFIG_APP_WEBCAM_CAMERA_QUALITY,

        /* 缓冲 */
        .fb_count = fb_count,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = grab_mode,
    };

    esp_err_t ret = esp_camera_init(&config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed %s", esp_err_to_name(ret));
        return ret;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s)
    {
        ESP_LOGI(TAG, "Detected sensor PID=0x%04X, VER=0x%02X",
                 s->id.PID, s->id.VER);
        configure_sensor(s);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to get sensor handle");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Camera ready!");
    return ESP_OK;
}
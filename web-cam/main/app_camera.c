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

    /* ---------------------------------------------------------
       [公共基础配置] - 适用于所有传感器的基础画质底座
       --------------------------------------------------------- */
    // --- 画面基础 ---
    s->set_brightness(s, 0); // 亮度调节：范围 [-2, 2]。0 为默认。
    s->set_contrast(s, 0);   // 对比度调节：范围 [-2, 2]。0 为默认。
    s->set_saturation(s, 0); // 饱和度调节：范围 [-2, 2]。0 为默认。
    s->set_hmirror(s, 0);    // 水平镜像：0关闭，1开启 (常用于前置摄像头效果)
    s->set_vflip(s, 0);      // 垂直翻转：0关闭，1开启 (根据摄像头物理安装方向决定)

    // --- 色彩与白平衡 ---
    s->set_whitebal(s, 1); // 自动白平衡 (AWB) 开关：1开启。必须开，否则偏色。
    s->set_awb_gain(s, 1); // 自动白平衡增益：1开启。允许算法根据色温调整 RGB 增益。
    s->set_wb_mode(s, 0);  // 白平衡模式：0为Auto。其他数值对应Sunny/Cloudy等。

    // --- 曝光与增益 (Auto) ---
    s->set_exposure_ctrl(s, 1); // 自动曝光控制 (AEC)：1开启。让传感器根据环境亮度自动调快门。
    s->set_aec2(s, 1);          // DSP高级自动曝光：1开启。使用数字信号处理器进行更平滑的曝光过渡。
    s->set_gain_ctrl(s, 1);     // 自动增益控制 (AGC)：1开启。暗光下自动提高ISO。

    // --- 基础画质修复 ---
    s->set_bpc(s, 1);     // 坏点校正 (Black Pixel Correction)：1开启。硬件剔除死像素。
    s->set_wpc(s, 1);     // 白点校正 (White Pixel Correction)：1开启。硬件剔除画面闪烁噪点。
    s->set_raw_gma(s, 1); // 伽马校正 (Gamma)：1开启。提亮暗部细节，使色彩映射符合人眼感知。
    s->set_lenc(s, 1);    // 镜头暗角补偿 (Lens Correction)：1开启。修复镜头边缘进光量少导致的四周发暗。

    /* ---------------------------------------------------------
       [传感器专属优化] - 根据不同芯片的物理特性扬长避短
       --------------------------------------------------------- */
    switch (s->id.PID)
    {
    /* ====== OV2640 (200万像素，ESP32-CAM最常见) ====== */
    case OV2640_PID:
        ESP_LOGI(TAG, "Sensor detected: OV2640");
        // 【特性】进光量小，动态范围极窄，暗光极差，色彩易偏红/偏暖。

        // 1. 解锁最大增益：因为底太小，暗光必须允许最高增益，否则全黑。
        s->set_gainceiling(s, (gainceiling_t)6); // 增益上限设为最高 (6)
        // 2. 色彩压制：稍微降低饱和度，防止在白炽灯下画面过红。
        s->set_saturation(s, -1);
        break;

    /* ====== OV3660 (300万像素) ====== */
    case OV3660_PID:
        ESP_LOGI(TAG, "Sensor detected: OV3660");
        // 【特性】动态范围较好，视角常较宽，但出厂默认设置极容易偏绿/偏青，画面发灰。

        // 1. 增益上限适中：其感光好于2640，设为5即可，避免噪点过多。
        s->set_gainceiling(s, (gainceiling_t)5);
        // 2. 提升通透度：增加一点亮度和对比度，去除“发灰”感。
        s->set_brightness(s, 1);
        s->set_contrast(s, 1);
        // 3. 物理翻转：市面上大部分给ESP32设计的OV3660模组，排线是反过来的，默认必须翻转。
        s->set_vflip(s, 1);
        break;

    /* ====== OV5640 (500万像素，支持自动对焦) ====== */
    case OV5640_PID:
        ESP_LOGI(TAG, "Sensor detected: OV5640");
        // 【特性】像素最高，画质最好。但像素密度高导致单像素面积小，需要极佳的光照；功耗大。

        // 1. 解决过暗问题 (核心调优)：
        // - 必须放开增益天花板，允许传感器在室内大幅拉高ISO。
        s->set_gainceiling(s, (gainceiling_t)6);
        // - 提升自动曝光级别 (AE Level)：取值 [-2, 2]。默认0。设为1或2，强制算法让画面更亮。
        s->set_ae_level(s, 2);
        s->set_brightness(s, 2);

        // 2. 图像质量：5MP下噪点容易被放大，开启高级降采样抗锯齿（取代不存在的set_denoise）
        s->set_dcw(s, 1); // Downsize Center Weight 开，缩小图像时能有效平滑噪点
        break;

    default:
        ESP_LOGW(TAG, "Unknown sensor PID: 0x%04X, using defaults", s->id.PID);
        s->set_gainceiling(s, (gainceiling_t)5);
        break;
    }

    /* ---------------------------------------------------------
       [分辨率自适应调优]
       --------------------------------------------------------- */
    framesize_t fs = s->status.framesize; // 获取当前设定的分辨率

    // CONFIG_APP_WEBCAM_CAMERA_QUALITY: ESP32中 JPEG 质量设置 (0-63)。
    // 【注意】数值越小，压缩率越低，画质越好，但文件越大，帧率越低！
    int base_quality = CONFIG_APP_WEBCAM_CAMERA_QUALITY;

    if (fs >= FRAMESIZE_UXGA) // 大于 1600x1200
    {
        // 高分辨率场景：由于SPI总线带宽限制，帧率本来就只有几帧，不如索性拉满画质
        s->set_quality(s, base_quality); // 保持高画质
        s->set_dcw(s, 1);                // 开启降采样中心权重算法，提升画面细腻度
    }
    else // 小于 UXGA (如 VGA 640x480, CIF 等)
    {
        // 低分辨率场景：通常用于视频流推流，优先保证帧率 (FPS)
        // 适当增大 quality 数值(如+4)，牺牲一点画质来减小JPEG体积，提升网络传输速度
        s->set_quality(s, base_quality + 4);
        s->set_dcw(s, 0); // 关闭 DCW，减轻 ISP (图像处理单元) 负担，提升处理速度
    }
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
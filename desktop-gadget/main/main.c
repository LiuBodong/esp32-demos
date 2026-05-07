/**
 * @file main.c
 * @brief 桌面摆件主程序 - 网络授时时钟
 *
 * 功能概述：
 *   1. 初始化 I2C 总线，驱动 SSD1315 OLED 显示屏（128x64）
 *   2. 通过 LVGL 图形库在 OLED 上绘制时钟界面
 *   3. 连接 WiFi 网络，通过 NTP 服务器获取精确时间
 *   4. 每秒刷新屏幕，显示当前日期和时间
 *
 * 硬件平台：ESP32-C3 + SSD1315 OLED (I2C, 128x64)
 * 软件框架：ESP-IDF 6.1.0 + LVGL 9.5.0 + esp_lvgl_port 2.7.2
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

/* ── FreeRTOS 头文件 ─────────────────────────────────────── */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/* ── ESP-IDF 系统头文件 ──────────────────────────────────── */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

/* ── I2C / LCD 驱动头文件 ────────────────────────────────── */
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"

/* ── LVGL 头文件 ─────────────────────────────────────────── */
#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "sdkconfig.h"

/* ═══════════════════════════════════════════════════════════
 *  硬件引脚与 I2C 配置宏定义
 * ═══════════════════════════════════════════════════════════ */

/** @brief I2C 端口号，ESP32-C3 有两个 I2C 控制器可选 */
#define I2C_HOST I2C_NUM_0

/** @brief I2C SDA（数据线）引脚编号，GPIO8 */
#define I2C_SDA GPIO_NUM_8

/** @brief I2C SCL（时钟线）引脚编号，GPIO9 */
#define I2C_SCL GPIO_NUM_9

/** @brief I2C 时钟频率 400kHz（Fast-Mode），标准模式为 100kHz */
#define I2C_FREQ_HZ 400000

/* ═══════════════════════════════════════════════════════════
 *  SSD1315 OLED 显示屏参数
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief OLED I2C 设备地址
 * SA0 引脚为 LOW 时地址为 0x3C，为 HIGH 时地址为 0x3D
 * 大多数模块默认 SA0=LOW，即 0x3C
 */
#define OLED_I2C_ADDR 0x3C

/** @brief OLED 显示屏宽度（像素） */
#define OLED_WIDTH 128

/** @brief OLED 显示屏高度（像素） */
#define OLED_HEIGHT 64

/* ═══════════════════════════════════════════════════════════
 *  WiFi 连接配置（通过 menuconfig 可配置）
 * ═══════════════════════════════════════════════════════════ */

/** @brief WiFi SSID（网络名称），在 menuconfig → WiFi Configuration 中配置 */
#define WIFI_SSID CONFIG_APP_DESKTOP_GADGET_WIFI_SSID

/** @brief WiFi 密码，在 menuconfig → WiFi Configuration 中配置 */
#define WIFI_PASSWORD CONFIG_APP_DESKTOP_GADGET_WIFI_PASSWORD

/**
 * @brief WiFi 连接最大重试次数
 * 超过此次数后 WiFi 任务将停止尝试
 */
#define WIFI_MAX_RETRY 10

/* ═══════════════════════════════════════════════════════════
 *  FreeRTOS 事件组位定义
 *  用于 WiFi 和 NTP 任务间的同步通知
 * ═══════════════════════════════════════════════════════════ */

/** @brief WiFi 已成功获取 IP 地址事件位 */
#define WIFI_CONNECTED_BIT BIT0

/** @brief WiFi 连接失败事件位 */
#define WIFI_FAIL_BIT BIT1

/** @brief NTP 时间同步完成事件位 */
#define NTP_SYNCED_BIT BIT2

/* ═══════════════════════════════════════════════════════════
 *  时钟界面布局常量
 * ═══════════════════════════════════════════════════════════ */

/** @brief 时间显示区域 Y 坐标偏移（像素），位于屏幕上半部分 */
#define CLOCK_TIME_Y_OFFSET -16

/** @brief 日期显示区域 Y 坐标偏移（像素），位于屏幕下半部分 */
#define CLOCK_DATE_Y_OFFSET 8

/** @brief 状态信息 Y 坐标偏移（像素），位于屏幕底部 */
#define CLOCK_STATUS_Y_OFFSET 24

/* ═══════════════════════════════════════════════════════════
 *  全局变量
 * ═══════════════════════════════════════════════════════════ */

/** @brief 日志标签，用于 ESP_LOGx 宏输出 */
static const char *TAG = "desktop-gadget";

/**
 * @brief FreeRTOS 事件组句柄
 * 用于协调 WiFi 连接和 NTP 同步的事件通知
 */
static EventGroupHandle_t s_wifi_event_group;

/** @brief WiFi 连接重试计数器 */
static int s_retry_num = 0;

/**
 * @brief LVGL UI 控件指针
 * 使用全局变量以便在 LVGL 定时器回调中更新显示内容
 */
static lv_obj_t *s_time_label = NULL;   /**< 时间显示标签 (HH:MM:SS) */
static lv_obj_t *s_date_label = NULL;   /**< 日期显示标签 (YYYY-MM-DD Weekday) */
static lv_obj_t *s_status_label = NULL; /**< 状态信息标签 (WiFi/NTP 状态) */

/* ═══════════════════════════════════════════════════════════
 *  WiFi 事件处理回调函数
 *
 *  处理以下事件：
 *    - WIFI_EVENT_STA_START:       WiFi STA 模式启动，开始连接
 *    - WIFI_EVENT_STA_DISCONNECTED: WiFi 断开连接，根据重试次数决定是否重连
 *    - IP_EVENT_STA_GOT_IP:        成功获取 IP 地址，WiFi 连接完成
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief WiFi 和 IP 事件统一处理回调
 *
 * @param arg       用户自定义参数（未使用，传入 NULL）
 * @param event_base 事件基类型（WIFI_EVENT 或 IP_EVENT）
 * @param event_id   事件 ID
 * @param event_data 事件附加数据
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    /* ── WiFi STA 模式启动事件 ───────────────────────────── */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        /* WiFi STA 模式已启动，发起首次连接请求 */
        ESP_LOGI(TAG, "WiFi STA 启动，开始连接...");
        esp_wifi_connect();
    }
    /* ── WiFi 断开连接事件 ───────────────────────────────── */
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_MAX_RETRY)
        {
            /* 未超过最大重试次数，等待 1 秒后重新连接 */
            ESP_LOGW(TAG, "WiFi 连接断开，正在重试... (%d/%d)",
                     s_retry_num + 1, WIFI_MAX_RETRY);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_wifi_connect();
            s_retry_num++;
        }
        else
        {
            /* 超过最大重试次数，设置失败标志位 */
            ESP_LOGE(TAG, "WiFi 连接失败，已达最大重试次数");
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    /* ── 获取 IP 地址事件 ────────────────────────────────── */
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        /* 成功获取 IP 地址，打印地址信息并清除失败标志 */
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi 已连接，IP 地址: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ═══════════════════════════════════════════════════════════
 *  WiFi 初始化与连接函数
 *
 *  执行流程：
 *    1. 初始化 NVS（非易失性存储，WiFi 驱动需要）
 *    2. 初始化网络接口和事件循环
 *    3. 配置 WiFi STA 模式（站点模式）
 *    4. 启动 WiFi 并等待连接结果
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief 初始化 WiFi 并连接到配置的网络
 *
 * 该函数会阻塞当前任务，直到 WiFi 连接成功或失败。
 * 连接参数通过 menuconfig 中的 CONFIG_APP_DESKTOP_GADGET_WIFI_SSID 和 CONFIG_APP_DESKTOP_GADGET_WIFI_PASSWORD 配置。
 *
 * @return
 *     - ESP_OK: WiFi 连接成功
 *     - ESP_FAIL: WiFi 连接失败（超过最大重试次数）
 */
static esp_err_t wifi_init_and_connect(void)
{
    /* ── 第 1 步：初始化 NVS（WiFi 驱动依赖此存储） ──────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        /* NVS 分区格式不兼容，擦除后重新初始化 */
        ESP_LOGW(TAG, "NVS 分区格式不兼容，正在擦除...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── 第 2 步：创建事件组用于任务同步 ─────────────────── */
    s_wifi_event_group = xEventGroupCreate();

    /* ── 第 3 步：初始化 TCP/IP 网络栈 ───────────────────── */
    ESP_ERROR_CHECK(esp_netif_init());

    /* ── 第 4 步：创建默认事件循环 ───────────────────────── */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ── 第 5 步：创建默认 WiFi STA 网络接口 ─────────────── */
    esp_netif_create_default_wifi_sta();

    /* ── 第 6 步：初始化 WiFi 驱动，使用默认配置 ─────────── */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* ── 第 7 步：注册 WiFi 和 IP 事件处理回调 ───────────── */
    /* 注册 WiFi 事件回调（处理 STA_START、STA_DISCONNECTED 等） */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    /* 注册 IP 事件回调（处理 GOT_IP 事件） */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    /* ── 第 8 步：配置 WiFi STA 连接参数 ─────────────────── */
    wifi_config_t wifi_config = {
        .sta = {
            /* SSID 和密码通过 menuconfig 配置，编译时写入 */
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            /* 设置最低认证阈值为 OPEN，兼容所有认证模式的路由器 */
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };

    /* ── 第 9 步：设置 WiFi 模式为 STA 并启动 ────────────── */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(8)); // 设置最大发射功率为 8dBm（即 6mW）
    ESP_LOGI(TAG, "已降低 WiFi 发射功率"); 

    ESP_LOGI(TAG, "WiFi 初始化完成，SSID: %s, PASSWORD: %s", WIFI_SSID, WIFI_PASSWORD);

    /* ── 第 10 步：等待连接结果（阻塞） ──────────────────── */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,      /* 不清除事件位 */
        pdFALSE,      /* 不等待所有位，任一即可 */
        portMAX_DELAY /* 无限等待 */
    );

    /* ── 第 11 步：根据事件位判断连接结果 ────────────────── */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "WiFi 连接成功");
        return ESP_OK;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "WiFi 连接失败");
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGE(TAG, "WiFi 连接出现意外状态");
        return ESP_FAIL;
    }
}

/* ═══════════════════════════════════════════════════════════
 *  NTP 时间同步回调函数
 *
 *  当 SNTP 模块成功从 NTP 服务器获取到时间后，此回调被调用。
 *  回调中设置 NTP_SYNCED_BIT 事件位，通知主任务时间已同步。
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief NTP 时间同步回调函数
 *
 * @param tv 指向 timeval 结构体的指针，包含同步到的时间
 */
static void ntp_sync_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP 时间同步完成! 时间: %ld", (long)tv->tv_sec);
    xEventGroupSetBits(s_wifi_event_group, NTP_SYNCED_BIT);
}

/* ═══════════════════════════════════════════════════════════
 *  NTP 初始化与时间同步函数
 *
 *  使用 ESP-IDF 的 SNTP 组件从配置的 NTP 服务器获取时间。
 *  SNTP（Simple Network Time Protocol）是 NTP 的简化版本，
 *  适用于嵌入式设备。
 *
 *  时区信息通过 setenv("TZ", ...) 设置，支持 POSIX 时区字符串。
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief 初始化 SNTP 并同步网络时间
 *
 * 该函数会阻塞当前任务，直到 NTP 时间同步完成。
 * NTP 服务器地址通过 menuconfig 中的 CONFIG_APP_DESKTOP_GADGET_NTP_SERVER1/2/3 配置。
 * 时区偏移通过 CONFIG_APP_DESKTOP_GADGET_NTP_GMT_OFFSET 和 CONFIG_APP_DESKTOP_GADGET_NTP_DAYLIGHT_OFFSET 配置。
 *
 * @return
 *     - ESP_OK: NTP 时间同步成功
 *     - ESP_FAIL: NTP 时间同步超时（60 秒）
 */
static esp_err_t ntp_init_and_sync(void)
{
    ESP_LOGI(TAG, "正在初始化 NTP 时间同步...");

    /* ── 第 1 步：设置时区 ───────────────────────────────── */
    /* 根据 menuconfig 配置的 GMT 偏移量构造 POSIX 时区字符串 */
    /* 例如：GMT+8 对应 "CST-8" */
    char tz_str[16];
    int gmt_offset = CONFIG_APP_DESKTOP_GADGET_NTP_GMT_OFFSET;
    int dst_offset = CONFIG_APP_DESKTOP_GADGET_NTP_DAYLIGHT_OFFSET;

    if (gmt_offset >= 0)
    {
        /* 正偏移（东半球），如 UTC+8 → "CST-8" */
        snprintf(tz_str, sizeof(tz_str), "CST%d", -gmt_offset / 3600);
    }
    else
    {
        /* 负偏移（西半球），如 UTC-5 → "EST5" */
        snprintf(tz_str, sizeof(tz_str), "EST%d", -gmt_offset / 3600);
    }

    /* 如果有夏令时偏移，追加 DST 规则 */
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

    /* ── 第 2 步：配置 SNTP ─────────────────────────────── */
    /* 使用 ESP-IDF 内建的 SNTP 模块 */
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);

    /* 设置 NTP 服务器地址（最多 3 个，优先使用第一个） */
    esp_sntp_setservername(0, CONFIG_APP_DESKTOP_GADGET_NTP_SERVER1);
    esp_sntp_setservername(1, CONFIG_APP_DESKTOP_GADGET_NTP_SERVER2);
    esp_sntp_setservername(2, CONFIG_APP_DESKTOP_GADGET_NTP_SERVER3);

    /* 注册同步完成回调函数 */
    esp_sntp_set_time_sync_notification_cb(ntp_sync_cb);

    /* 设置同步间隔为 1 小时（3600000 毫秒） */
    esp_sntp_set_sync_interval(3600000);

    /* ── 第 3 步：启动 SNTP 服务 ────────────────────────── */
    esp_sntp_init();

    ESP_LOGI(TAG, "SNTP 已启动，正在等待时间同步...");
    ESP_LOGI(TAG, "NTP 服务器: %s, %s, %s",
             CONFIG_APP_DESKTOP_GADGET_NTP_SERVER1, CONFIG_APP_DESKTOP_GADGET_NTP_SERVER2, CONFIG_APP_DESKTOP_GADGET_NTP_SERVER3);

    /* ── 第 4 步：等待 NTP 同步完成（带超时） ────────────── */
    /* 超时时间 60 秒，如果网络状况差可能需要更长时间 */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        NTP_SYNCED_BIT,
        pdFALSE,             /* 不清除事件位 */
        pdFALSE,             /* 不等待所有位 */
        pdMS_TO_TICKS(60000) /* 60 秒超时 */
    );

    if (bits & NTP_SYNCED_BIT)
    {
        /* NTP 同步成功，打印当前时间 */
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
        ESP_LOGE(TAG, "NTP 时间同步超时（60秒）");
        return ESP_FAIL;
    }
}

/* ═══════════════════════════════════════════════════════════
 *  SSD1315 OLED 屏幕初始化函数
 *
 *  初始化流程：
 *    1. 配置并初始化 I2C 主机总线
 *    2. 创建 LCD Panel IO（I2C 协议层）
 *    3. 创建 SSD1306 面板驱动（兼容 SSD1315）
 *    4. 初始化 LVGL 图形库和显示端口
 *    5. 注册 OLED 为 LVGL 显示设备
 *
 *  关于 SSD1315 与 SSD1306：
 *    SSD1315 是 SSD1306 的改进版本，寄存器完全兼容，
 *    因此使用 esp_lcd 提供的 SSD1306 驱动即可正常工作。
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief 初始化 SSD1315 OLED 显示屏和 LVGL 图形库
 *
 * 完成后，LVGL 将拥有一个 128x64 的单色显示设备，
 * 可以通过 lvgl_port_lock()/lvgl_port_unlock() 线程安全地操作。
 */
static void screen_init(void)
{
    /* ────────────────────────────────────────────────────
     *  第 1 步：初始化 I2C 主机总线
     *
     *  I2C 总线是连接 ESP32-C3 和 SSD1315 的通信接口。
     *  使用新的 i2c_master_bus API（ESP-IDF 5.x+），支持多设备共享总线。
     * ──────────────────────────────────────────────────── */
    i2c_master_bus_handle_t bus_handle = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,    /* 使用默认时钟源 */
        .i2c_port = I2C_HOST,                 /* I2C 端口号 */
        .sda_io_num = I2C_SDA,                /* SDA 数据引脚 */
        .scl_io_num = I2C_SCL,                /* SCL 时钟引脚 */
        .glitch_ignore_cnt = 7,               /* 毛刺滤波计数，抗干扰 */
        .flags.enable_internal_pullup = true, /* 启用内部上拉电阻 */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    ESP_LOGI(TAG, "I2C 总线初始化完成 (SDA=%d, SCL=%d, Freq=%dHz)",
             I2C_SDA, I2C_SCL, I2C_FREQ_HZ);

    /* ────────────────────────────────────────────────────
     *  第 2 步：创建 LCD Panel IO（I2C 协议层）
     *
     *  Panel IO 负责 I2C 通信的协议细节：
     *  - dev_addr:        I2C 设备地址（0x3C）
     *  - control_phase_bytes: 控制阶段字节数（1 字节）
     *  - lcd_cmd_bits:    命令位数（8 位）
     *  - lcd_param_bits:  参数位数（8 位）
     *  - dc_bit_offset:   D/C# 位在控制字节中的位置
     *    SSD1306/SSD1315 的 I2C 控制字节格式：
     *    Bit[7:6] = 00: Co=0, D/C#=0 → 后续全部为命令
     *    Bit[7:6] = 01: Co=0, D/C#=1 → 后续全部为数据
     *    dc_bit_offset=6 表示 D/C# 位在 bit[6]
     * ──────────────────────────────────────────────────── */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = OLED_I2C_ADDR,   /* I2C 设备地址 */
        .control_phase_bytes = 1,    /* 控制阶段 1 字节 */
        .lcd_cmd_bits = 8,           /* 命令 8 位 */
        .lcd_param_bits = 8,         /* 参数 8 位 */
        .dc_bit_offset = 6,          /* D/C# 位偏移 */
        .scl_speed_hz = I2C_FREQ_HZ, /* I2C 时钟频率 */
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus_handle, &io_cfg, &io_handle));

    /* ────────────────────────────────────────────────────
     *  第 3 步：创建 SSD1306 面板驱动实例
     *
     *  配置面板参数：
     *  - bits_per_pixel: 1（单色 OLED，每个像素 1 位）
     *  - reset_gpio_num: -1（无硬件复位引脚，使用软件复位）
     *
     *  创建驱动后依次执行：
     *  1. reset:   复位面板（发送复位命令）
     *  2. init:    初始化面板（配置寄存器，清屏）
     *  3. disp_on: 开启显示
     * ──────────────────────────────────────────────────── */
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .bits_per_pixel = 1,  /* 单色 1 bpp */
        .reset_gpio_num = -1, /* 无硬件复位引脚 */
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_cfg, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    /* 如需上下翻转显示内容，取消下面这行的注释 */
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "SSD1315 OLED 初始化完成 (%dx%d)", OLED_WIDTH, OLED_HEIGHT);

    /* ────────────────────────────────────────────────────
     *  第 4 步：初始化 LVGL 图形库
     *
     *  esp_lvgl_port 是 Espressif 提供的 LVGL 移植层，
     *  它会自动创建 LVGL 任务（FreeRTOS task）来处理：
     *  - LVGL 定时器（处理动画、过渡等）
     *  - 显示刷新（将 LVGL 缓冲区数据发送到屏幕）
     *  - 输入设备轮询（触摸、编码器等）
     *
     *  默认配置：
     *  - task_priority = 4
     *  - task_stack = 7168 字节
     *  - timer_period_ms = 5（200Hz 刷新率）
     * ──────────────────────────────────────────────────── */
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    /* ────────────────────────────────────────────────────
     *  第 5 步：将 OLED 注册为 LVGL 显示设备
     *
     *  显示配置：
     *  - buffer_size:       全屏缓冲（128*64=8192 像素）
     *  - double_buffer:     禁用双缓冲（节省内存）
     *  - monochrome:        标记为单色显示器
     *  - color_format:      RGB565（LVGL 内部格式，驱动会自动转换为 1bpp）
     *  - rotation.mirror_x/y: 水平/垂直镜像（根据实际屏幕方向调整）
     * ──────────────────────────────────────────────────── */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,                  /* Panel IO 句柄 */
        .panel_handle = panel_handle,            /* 面板驱动句柄 */
        .buffer_size = OLED_WIDTH * OLED_HEIGHT, /* 全屏缓冲 */
        .double_buffer = false,                  /* 禁用双缓冲 */
        .hres = OLED_WIDTH,                      /* 水平分辨率 128 */
        .vres = OLED_HEIGHT,                     /* 垂直分辨率 64 */
        .monochrome = true,                      /* 单色显示器 */
        .color_format = LV_COLOR_FORMAT_RGB565,  /* LVGL 内部颜色格式 */
        .rotation = {
            .swap_xy = false, /* 不交换 XY 轴 */
            .mirror_x = true, /* 水平镜像（根据实际方向调整） */
            .mirror_y = true, /* 垂直镜像（根据实际方向调整） */
        },
    };
    lvgl_port_add_disp(&disp_cfg);
    ESP_LOGI(TAG, "LVGL 显示设备注册完成");
}

/* ═══════════════════════════════════════════════════════════
 *  LVGL 时钟 UI 创建与更新函数
 *
 *  时钟界面布局（128x64 OLED）：
 *  ┌────────────────────────────────┐
 *  │                                │
 *  │        14:30:25  (时间)         │  ← 大号字体，居中
 *  │                                │
 *  │   2026-05-07 周三  (日期)       │  ← 小号字体，居中
 *  │                                │
 *  │      WiFi OK / NTP OK          │  ← 最小字体，状态栏
 *  │                                │
 *  └────────────────────────────────┘
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief LVGL 定时器回调函数 - 每秒更新时钟显示
 *
 * 此函数由 LVGL 定时器每秒调用一次，负责：
 *   1. 获取当前系统时间
 *   2. 格式化时间字符串（HH:MM:SS）
 *   3. 格式化日期字符串（YYYY-MM-DD 星期X）
 *   4. 更新 LVGL 标签控件的文本
 *
 * @param timer LVGL 定时器指针（由 lv_timer_create 自动传入）
 */
static void clock_update_timer_cb(lv_timer_t *timer)
{
    /* 获取当前系统时间（已通过 NTP 同步） */
    time_t now;
    struct tm time_info;
    time(&now);
    localtime_r(&now, &time_info);

    /* ── 格式化时间字符串 (HH:MM:SS) ────────────────────── */
    char time_buf[16];
    strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &time_info);
    lv_label_set_text(s_time_label, time_buf);

    /* ── 格式化日期字符串 (YYYY-MM-DD 周X) ──────────────── */
    char date_buf[32];
    /* 中文星期名称数组，tm_wday: 0=周日, 1=周一, ..., 6=周六 */
    const char *weekdays[] = {"周日", "周一", "周二", "周三", "周四", "周五", "周六"};
    snprintf(date_buf, sizeof(date_buf), "%04d-%02d-%02d %s",
             time_info.tm_year + 1900, /* tm_year 是从 1900 开始的偏移 */
             time_info.tm_mon + 1,     /* tm_mon 范围是 0-11 */
             time_info.tm_mday,
             weekdays[time_info.tm_wday]);
    lv_label_set_text(s_date_label, date_buf);
}

/**
 * @brief 创建时钟显示界面
 *
 * 在 LVGL 活动屏幕上创建三个标签控件：
 *   1. 时间标签 - 显示 HH:MM:SS，使用 16 号字体
 *   2. 日期标签 - 显示 YYYY-MM-DD 周X，使用 12 号字体
 *   3. 状态标签 - 显示 WiFi/NTP 状态，使用 12 号字体
 *
 * 并创建一个 LVGL 定时器，每 1000ms（1 秒）调用一次更新函数。
 */
static void clock_ui_create(void)
{
    /* ── 获取活动屏幕并设置背景色 ──────────────────────── */
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);

    /* ── 创建时间标签（大号，居中偏上） ─────────────────── */
    s_time_label = lv_label_create(scr);
    lv_obj_set_style_text_color(s_time_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_time_label, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(s_time_label, LV_ALIGN_CENTER, 0, CLOCK_TIME_Y_OFFSET);

    /* ── 创建日期标签（中号，居中偏下） ─────────────────── */
    s_date_label = lv_label_create(scr);
    lv_obj_set_style_text_color(s_date_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_date_label, &lv_font_source_han_sans_sc_16_cjk, LV_PART_MAIN);
    lv_obj_align(s_date_label, LV_ALIGN_CENTER, 0, CLOCK_DATE_Y_OFFSET);

    /* ── 创建状态标签（小号，底部） ─────────────────────── */
    s_status_label = lv_label_create(scr);
    lv_obj_set_style_text_color(s_status_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_label_set_text(s_status_label, "WIFI: OK | NTP: OK");
    lv_obj_align(s_status_label, LV_ALIGN_CENTER, 0, CLOCK_STATUS_Y_OFFSET);

    /* ── 创建 LVGL 定时器，每 1000ms 更新一次时钟 ──────── */
    lv_timer_create(clock_update_timer_cb, 1000, NULL);

    /* ── 立即执行一次更新，避免首次显示空白 ─────────────── */
    clock_update_timer_cb(NULL);

    ESP_LOGI(TAG, "时钟 UI 创建完成");
}

/* ═══════════════════════════════════════════════════════════
 *  应用程序入口函数
 *
 *  执行流程：
 *    1. 初始化 OLED 显示屏和 LVGL
 *    2. 连接 WiFi 网络
 *    3. 通过 NTP 同步网络时间
 *    4. 创建时钟 UI 并启动自动更新
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief ESP-IDF 应用程序入口点
 *
 * 程序启动后的主执行流程，所有初始化在此函数中顺序完成。
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  桌面摆件 - 网络授时时钟");
    ESP_LOGI(TAG, "  硬件: ESP32-C3 + SSD1315 OLED");
    ESP_LOGI(TAG, "========================================");

    /* ── 第 1 步：初始化 OLED 显示屏和 LVGL ─────────────── */
    ESP_LOGI(TAG, "[1/3] 初始化 OLED 显示屏...");
    screen_init();

    /* ── 第 2 步：连接 WiFi 网络 ─────────────────────────── */
    ESP_LOGI(TAG, "[2/3] 连接 WiFi 网络...");
    esp_err_t wifi_ret = wifi_init_and_connect();
    if (wifi_ret != ESP_OK)
    {
        /* WiFi 连接失败，在屏幕上显示错误信息 */
        ESP_LOGE(TAG, "WiFi 连接失败，无法进行 NTP 同步");

        lvgl_port_lock(0);
        lv_obj_t *scr = lv_screen_active();
        lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);

        lv_obj_t *err_label = lv_label_create(scr);
        lv_label_set_text(err_label, "WiFi Failed!");
        lv_obj_set_style_text_color(err_label, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(err_label, &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_align(err_label, LV_ALIGN_CENTER, 0, 0);
        lvgl_port_unlock();

        /* WiFi 失败后程序仍然继续运行，但不进行 NTP 同步 */
        return;
    }

    /* ── 第 3 步：通过 NTP 同步网络时间 ──────────────────── */
    ESP_LOGI(TAG, "[3/3] 同步 NTP 时间...");
    esp_err_t ntp_ret = ntp_init_and_sync();
    if (ntp_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "NTP 同步失败，将使用本地时间");
    }

    /* ── 第 4 步：创建时钟显示界面 ───────────────────────── */
    ESP_LOGI(TAG, "创建时钟界面...");
    lvgl_port_lock(0); /* 获取 LVGL 互斥锁（线程安全） */
    clock_ui_create();
    lvgl_port_unlock(); /* 释放 LVGL 互斥锁 */

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  桌面摆件启动完成！");
    ESP_LOGI(TAG, "========================================");
}

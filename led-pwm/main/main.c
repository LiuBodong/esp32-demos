#include "driver/ledc.h"
#include "esp_clk_tree.h"
#include "esp_err.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "sdkconfig.h"

static const char *TAG = "LED_PWM";

// =========== 宏提取 ===========
/**
 * LEDC_MODE
 * 速度模式选择：LEDC_LOW_SPEED_MODE 表示低速模式。
 * 低速模式在所有 ESP32 系列芯片（ESP32 / S2 / S3 / C3 等）上都可用，
 * 且通常满足一般 PWM 应用需求，同时具备更低的功耗。
 * 若需更高精度或更高频率，可使用 LEDC_HIGH_SPEED_MODE，但部分芯片可能不支持。
 */
#define LEDC_MODE LEDC_LOW_SPEED_MODE

/**
 * LEDC_TIMER
 * 使用的 LEDC 硬件定时器编号（0 ~ 3）。
 * 每个定时器可独立设置频率和分辨率，多个通道可共用同一个定时器。
 * 具体值来自 Kconfig 配置项 CONFIG_APP_LED_PWM_LEDC_TIMER。
 */
#define LEDC_TIMER CONFIG_APP_LED_PWM_LEDC_TIMER

/**
 * LEDC_FREQUENCE
 * PWM 输出频率，单位 Hz。
 * 人眼对于 LED 闪烁的感知阈值约为 50 ~ 60 Hz，因此通常设置频率大于 100 Hz
 * 以避免闪烁。 具体值来自 Kconfig 配置项
 * CONFIG_APP_LED_PWM_LEDC_PWM_FREQUENCY。
 */
#define LEDC_FREQUENCE CONFIG_APP_LED_PWM_LEDC_PWM_FREQUENCY

/**
 * LEDC_CHANNEL
 * LEDC 输出通道编号（0 ~ 7）。
 * 一个定时器可驱动多个通道，各通道可输出相同频率但不同占空比的 PWM 信号。
 * 具体值来自 Kconfig 配置项 CONFIG_APP_LED_PWM_LEDC_CHANNEL。
 */
#define LEDC_CHANNEL CONFIG_APP_LED_PWM_LEDC_CHANNEL

/**
 * LEDC_GPIO_PIN
 * 实际连接 LED 的 GPIO 引脚编号。
 * 需确保该引脚支持输出功能，且未与其他外设冲突。
 * 具体值来自 Kconfig 配置项 CONFIG_APP_LED_PWM_GPIO_PIN。
 */
#define LEDC_GPIO_PIN CONFIG_APP_LED_PWM_GPIO_PIN

/**
 * LEDC_FADE_TIME
 * 渐变时间，即从一个占空比值渐变到另一个占空比值所需的时间，单位毫秒。
 * 该值越大，呼吸灯亮度变化越平缓；越小则变化越急促。
 * 具体值来自 Kconfig 配置项 CONFIG_APP_LED_PWM_LEDC_FADE_TIME。
 */
#define LEDC_FADE_TIME CONFIG_APP_LED_PWM_LEDC_FADE_TIME

// =========== 全局变量：存储运行时确定的分辨率及占空比极值 ===========
/**
 * 由于分辨率是在运行时通过 ledc_find_suitable_duty_resolution 计算得出的，
 * 因此我们使用全局变量来保存实际使用的分辨率，以及由此计算出的最大占空比、
 * LED 亮起和熄灭对应的占空比值。这些值将在 app_init 中初始化，并在 app_main
 * 中使用。
 */
static uint32_t g_actual_duty_resolution = 0; // 实际采用的分辨率（单位：bit）
static uint32_t g_duty_max_value =
    0; // 最大占空比数值，即 (1 << resolution) - 1
static uint32_t g_duty_on_value  = 0; // LED 点亮时对应的占空比
static uint32_t g_duty_off_value = 0; // LED 熄灭时对应的占空比

/**
 * @brief LEDC 初始化函数：配置定时器、通道以及渐变功能。
 *        同时计算出硬件支持的最佳分辨率，并初始化占空比相关的全局变量。
 */
static esp_err_t app_init(void) {
    // 确定使用的时钟源
    ledc_clk_cfg_t clk_cfg = LEDC_AUTO_CLK;
    // 直接指定默认的时钟源频率 (APB_CLK 通常为 80 MHz)
    uint32_t src_clk_freq = 80 * 1000000;   // 80 MHz
    uint32_t target_freq  = LEDC_FREQUENCE; // 期望的 PWM 频率

    uint32_t suitable_resolution = 0;

    // ========== 根据 Kconfig 配置选择分辨率来源 ==========
#ifdef CONFIG_APP_LED_PWM_DUTY_RES_MANUAL
    /**
     * 手动分辨率模式：
     * 用户通过 menuconfig 直接指定了占空比分辨率（单位 bit）。
     * 直接读取配置值，但需验证该分辨率是否能在 80 MHz 时钟下产生目标频率。
     *
     * 验证公式：
     *   最大可输出频率 = 源时钟频率 / (2^分辨率)
     * 若目标频率大于最大可输出频率，则硬件无法实现，应报错退出。
     */
    suitable_resolution = CONFIG_APP_LED_PWM_LEDC_DUTY_RESOLUTION;

    // 计算该分辨率下理论最大频率
    uint32_t max_possible_freq = src_clk_freq / (1UL << suitable_resolution);
    if (target_freq > max_possible_freq) {
        ESP_LOGE(TAG,
                 "手动设置的分辨率 %d bit 无法支持 %d Hz 的频率！"
                 "该分辨率下最大频率为 %lu Hz。请降低频率或减小分辨率。",
                 (int)suitable_resolution, (int)target_freq,
                 (unsigned long)max_possible_freq);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "使用手动指定的分辨率: %d bit (最大可支持 %lu Hz)",
             (int)suitable_resolution, (unsigned long)max_possible_freq);
#else
    /**
     * 自动分辨率模式（默认）：
     * 调用 SDK 提供的 ledc_find_suitable_duty_resolution 函数，
     * 根据源时钟频率和目标 PWM 频率，自动计算出一个既能满足频率要求、
     * 又能提供最高占空比精度的分辨率。
     *
     * 该函数内部会遍历可能的分辨率值，返回最大的可行分辨率；
     * 若找不到任何可用分辨率，则返回 0。
     */
    suitable_resolution =
        ledc_find_suitable_duty_resolution(src_clk_freq, target_freq);

    if (suitable_resolution == 0) {
        ESP_LOGE(TAG, "无法为 %d Hz 的频率找到合适的分辨率，请降低期望频率。",
                 (int)target_freq);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "自动选择的分辨率: %d bit", (int)suitable_resolution);
#endif

    // ========== 通用初始化部分 ==========

    // 将最终使用的分辨率保存到全局变量，后续用于计算占空比极值
    g_actual_duty_resolution = suitable_resolution;
    // 根据分辨率计算最大占空比数值（例如 13 bit 分辨率时，最大值为 8191）
    g_duty_max_value = (1UL << g_actual_duty_resolution) - 1;

    // 根据 GPIO 有效电平配置，计算 LED 亮起和熄灭对应的占空比
#ifdef CONFIG_APP_LED_PWM_GPIO_ACTIVE_LOW
    /**
     * 低电平有效模式（LED 阴极接 GPIO，阳极接 VCC）：
     *   - GPIO 输出低电平时，LED 点亮。
     *   - GPIO 输出高电平时，LED 熄灭。
     * 因此：
     *   - 熄灭状态对应占空比 = g_duty_max_value（全高电平）。
     *   - 最亮状态对应占空比 = 0（全低电平）。
     */
    g_duty_off_value = g_duty_max_value;
    g_duty_on_value  = 0;
#else
    /**
     * 高电平有效模式（LED 阳极接 GPIO，阴极接 GND）：
     *   - GPIO 输出高电平时，LED 点亮。
     *   - GPIO 输出低电平时，LED 熄灭。
     * 因此：
     *   - 熄灭状态对应占空比 = 0。
     *   - 最亮状态对应占空比 = g_duty_max_value。
     */
    g_duty_off_value = 0;
    g_duty_on_value  = g_duty_max_value;
#endif

    ESP_LOGI(TAG, "占空比范围: 0 ~ %lu, 亮起值: %lu, 熄灭值: %lu",
             (unsigned long)g_duty_max_value, (unsigned long)g_duty_on_value,
             (unsigned long)g_duty_off_value);

    /**
     * 配置 LEDC 定时器
     *  - speed_mode      : LEDC 速度模式（通常使用低速模式，兼容性最好）
     *  - clk_cfg         : 时钟源，LEDC_AUTO_CLK 让驱动自动选择
     *  - timer_num       : 定时器编号
     *  - duty_resolution : 占空比分辨率（bit），这里使用我们确定的值
     *  - freq_hz         : PWM 输出频率
     */
    ledc_timer_config_t timer_config = {
        .speed_mode      = LEDC_MODE,
        .clk_cfg         = LEDC_AUTO_CLK,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = suitable_resolution, // 使用动态/手动确定的分辨率
        .freq_hz         = LEDC_FREQUENCE,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    /**
     * 配置 LEDC 输出通道
     *  - channel    : 通道编号
     *  - gpio_num   : 输出 PWM 的 GPIO
     *  - duty       : 初始占空比，设为熄灭状态，避免上电瞬间闪烁
     */
    ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LEDC_GPIO_PIN,
        .duty       = g_duty_off_value, // 初始化为熄灭状态
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    /**
     * 安装 LEDC 渐变功能
     * 参数 0 表示不使用中断通知，渐变由硬件自动完成。
     */
    ESP_ERROR_CHECK(ledc_fade_func_install(0));

    return ESP_OK;
}

/**
 * @brief 应用程序入口函数。
 * 初始化后进入无限循环，实现 LED 呼吸灯效果。
 */
void app_main(void) {
    // 执行初始化
    esp_err_t ret = app_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ERROR init! exit!");
        return;
    }

    while (1) {
        ESP_LOGI(TAG, "LED 逐渐变亮 (逻辑 0%% -> 100%%)");
        /**
         * ledc_set_fade_with_time()
         * 设置渐变目标占空比及渐变时长。
         * 参数：
         *   - speed_mode  : LEDC 速度模式。
         *   - channel     : 输出通道。
         *   - target_duty : 目标占空比数值（范围由分辨率决定）。
         *                   这里使用运行时计算的 g_duty_on_value。
         *   - max_fade_time_ms : 渐变过程持续时间，单位毫秒。
         * 该函数仅配置渐变参数，不立即启动渐变。
         */
        ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL,
                                g_duty_on_value, // 【修改】使用运行时变量
                                LEDC_FADE_TIME);

        /**
         * ledc_fade_start()
         * 启动之前设置的渐变过程。
         * 参数：
         *   - speed_mode : LEDC 速度模式。
         *   - channel    : 输出通道。
         *   - wait_mode  : 渐变等待模式。
         *       - LEDC_FADE_NO_WAIT   :
         * 函数立即返回，渐变在后台由硬件自动执行。
         *       - LEDC_FADE_WAIT_DONE : 函数阻塞，直到渐变完成才返回。
         */
        ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);

        /**
         * 延时等待渐变完成。
         * 由于使用了 LEDC_FADE_NO_WAIT，渐变在后台执行，此处通过 vTaskDelay
         * 等待与渐变时长相同的时间， 确保渐变过程结束后再进行下一步操作。
         * pdMS_TO_TICKS() 将毫秒转换为 FreeRTOS 系统节拍数。
         */
        vTaskDelay(pdMS_TO_TICKS(LEDC_FADE_TIME));

        ESP_LOGI(TAG, "LED 逐渐变暗 (逻辑 100%% -> 0%%)");
        // 设置目标占空比为熄灭状态，并启动渐变
        ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, g_duty_off_value,
                                LEDC_FADE_TIME);
        ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        vTaskDelay(pdMS_TO_TICKS(LEDC_FADE_TIME));
    }
}
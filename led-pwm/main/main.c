#include "driver/ledc.h"
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
 * LEDC_DUTY_RESOLUTION
 * 占空比分辨率，单位为位（bit）。
 * 例如 10 位分辨率表示占空比范围为 0 ~ (2^10 - 1) = 0 ~ 1023。
 * 分辨率越高，PWM 亮度调节越细腻，但最高可实现的 PWM 频率会相应降低。
 * 具体值来自 Kconfig 配置项 CONFIG_APP_LED_PWM_LEDC_DUTY_RES。
 */
#define LEDC_DUTY_RESOLUTION CONFIG_APP_LED_PWM_LEDC_DUTY_RES

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

// =========== 初始占空比与目标占空比计算 ===========
/**
 * DUTY_MAX_VALUE
 * 根据配置的分辨率计算出的最大占空比数值。
 * 例：若分辨率为 10 位，则 DUTY_MAX_VALUE = 1023。
 * 占空比数值将直接影响 LED 亮度：
 *   - 在低电平点亮模式下：占空比越大，LED 越暗（因高电平占比高）。
 *   - 在高电平点亮模式下：占空比越大，LED 越亮（因高电平占比高）。
 */
#define DUTY_MAX_VALUE ((1 << LEDC_DUTY_RESOLUTION) - 1)

#ifdef CONFIG_APP_LED_PWM_GPIO_ACTIVE_LOW
/**
 * 低电平有效模式（LED 阴极接 GPIO，阳极接 VCC）：
 *   - GPIO 输出低电平时，LED 两端有电压差，LED 点亮。
 *   - GPIO 输出高电平时，LED 两端无电压差或反偏，LED 熄灭。
 *
 * 因此：
 *   - 熄灭状态对应占空比 = DUTY_MAX_VALUE（全高电平）。
 *   - 最亮状态对应占空比 = 0（全低电平）。
 */
#define LED_DUTY_OFF_VALUE DUTY_MAX_VALUE
#define LED_DUTY_ON_VALUE (0)
#else
/**
 * 高电平有效模式（LED 阳极接 GPIO，阴极接 GND）：
 *   - GPIO 输出高电平时，LED 点亮。
 *   - GPIO 输出低电平时，LED 熄灭。
 *
 * 因此：
 *   - 熄灭状态对应占空比 = 0。
 *   - 最亮状态对应占空比 = DUTY_MAX_VALUE。
 */
#define LED_DUTY_OFF_VALUE (0)
#define LED_DUTY_ON_VALUE DUTY_MAX_VALUE
#endif

/**
 * @brief LEDC 初始化函数：配置定时器、通道以及渐变功能。
 */
static void app_init(void) {
    /**
     * ledc_timer_config_t 结构体配置：
     *   - speed_mode      : LEDC 速度模式（低速或高速）。
     *   - clk_cfg         : 时钟源配置。LEDC_AUTO_CLK
     * 表示由驱动程序自动选择合适时钟源。
     *   - timer_num       : 定时器编号，范围为 0 ~ 3。
     *   - duty_resolution : 占空比分辨率，单位为 bit。
     *   - freq_hz         : PWM 频率，单位 Hz。
     */
    ledc_timer_config_t timer_config = {
        .speed_mode      = LEDC_MODE,
        .clk_cfg         = LEDC_AUTO_CLK,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RESOLUTION,
        .freq_hz         = LEDC_FREQUENCE,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    /**
     * ledc_channel_config_t 结构体配置：
     *   - speed_mode : 与定时器保持一致的速度模式。
     *   - channel    : 输出通道编号，范围为 0 ~ 7。
     *   - timer_sel  : 指定该通道关联的定时器编号。
     *   - intr_type  : 中断类型。LEDC_INTR_DISABLE 表示不使用中断。
     *   - gpio_num   : 输出 PWM 信号的 GPIO 引脚。
     *   - duty       : 初始占空比数值。此处设为熄灭状态，避免上电瞬间闪烁。
     *   - hpoint     :
     * 高电平起始点偏移量（仅在高速模式下有意义，低速模式忽略，填 0 即可）。
     */
    ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LEDC_GPIO_PIN,
        .duty       = LED_DUTY_OFF_VALUE,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    /**
     * 安装 LEDC 渐变功能。
     * 参数 0 表示不使用中断通知，渐变在后台由硬件自动完成。
     * 若需在渐变结束时得到通知，可传入非零值并注册相应回调。
     */
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}

/**
 * @brief 应用程序入口函数。
 * 初始化后进入无限循环，实现 LED 呼吸灯效果。
 */
void app_main(void) {
    // 执行初始化
    app_init();

    while (1) {
        ESP_LOGI(TAG, "LED 逐渐变亮 (逻辑 0%% -> 100%%)");
        /**
         * ledc_set_fade_with_time()
         * 设置渐变目标占空比及渐变时长。
         * 参数：
         *   - speed_mode  : LEDC 速度模式。
         *   - channel     : 输出通道。
         *   - target_duty : 目标占空比数值（范围由分辨率决定）。
         *   - max_fade_time_ms : 渐变过程持续时间，单位毫秒。
         * 该函数仅配置渐变参数，不立即启动渐变。
         */
        ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, LED_DUTY_ON_VALUE,
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
        ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, LED_DUTY_OFF_VALUE,
                                LEDC_FADE_TIME);
        ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
        vTaskDelay(pdMS_TO_TICKS(LEDC_FADE_TIME));
    }
}
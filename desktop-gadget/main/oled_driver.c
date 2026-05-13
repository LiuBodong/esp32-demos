/**
 * @file oled_driver.c
 * @brief OLED屏幕驱动模块实现
 *
 * 处理SSD1315 OLED初始化和LVGL配置
 */

#include "oled_driver.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "sdkconfig.h"

static const char *TAG = "oled_driver";

/** @brief I2C端口号 */
#define I2C_HOST I2C_NUM_0

/** @brief I2C SDA引脚编号 */
#define I2C_SDA GPIO_NUM_8

/** @brief I2C SCL引脚编号 */
#define I2C_SCL GPIO_NUM_9

/** @brief I2C时钟频率 */
#define I2C_FREQ_HZ 400000

/** @brief OLED I2C设备地址 */
#define OLED_I2C_ADDR 0x3C

/** @brief OLED显示屏宽度 */
#define OLED_WIDTH 128

/** @brief OLED显示屏高度 */
#define OLED_HEIGHT 64

esp_err_t oled_driver_init(void)
{
    /* 初始化I2C主机总线 */
    i2c_master_bus_handle_t bus_handle = NULL;
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_HOST,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
    ESP_LOGI(TAG, "I2C总线初始化完成 (SDA=%d, SCL=%d, Freq=%dHz)",
             I2C_SDA, I2C_SCL, I2C_FREQ_HZ);

    /* 创建LCD Panel IO */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = OLED_I2C_ADDR,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 6,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus_handle, &io_cfg, &io_handle));

    /* 创建SSD1306面板驱动 */
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_cfg, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "SSD1315 OLED初始化完成 (%dx%d)", OLED_WIDTH, OLED_HEIGHT);

    /* 初始化LVGL图形库 */
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    /* 将OLED注册为LVGL显示设备 */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = OLED_WIDTH * OLED_HEIGHT,
        .double_buffer = false,
        .hres = OLED_WIDTH,
        .vres = OLED_HEIGHT,
        .monochrome = true,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
    };
    lvgl_port_add_disp(&disp_cfg);
    ESP_LOGI(TAG, "LVGL显示设备注册完成");

    return ESP_OK;
}
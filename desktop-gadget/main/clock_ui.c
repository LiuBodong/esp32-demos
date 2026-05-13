/**
 * @file clock_ui.c
 * @brief 时钟界面UI模块实现 - 现代化设计
 *
 * 设计特点：
 *   - 简洁现代的布局
 *   - 清晰的视觉层次
 *   - 优雅的分隔线装饰
 *   - 优化的字体搭配
 */

#include "clock_ui.h"

#include <stdio.h>
#include <time.h>
#include "esp_log.h"
#include "lvgl.h"

static const char *TAG = "clock_ui";

/* ── 布局常量 ──────────────────────────────────────────────── */

/** @brief 时间显示Y坐标（居中偏上） */
#define TIME_Y_OFFSET -18

/** @brief 分隔线Y坐标 */
#define SEPARATOR_Y_OFFSET 2

/** @brief 日期显示Y坐标 */
#define DATE_Y_OFFSET 16

/** @brief 时间字体 */
#define TIME_FONT &lv_font_montserrat_28

/** @brief 日期字体（中文） */
#define DATE_FONT &lv_font_source_han_sans_sc_16_cjk

/* ── 颜色定义 ──────────────────────────────────────────────── */

/** @brief 主要文字颜色（纯白） */
#define COLOR_TEXT_PRIMARY lv_color_white()

/** @brief 次要文字颜色（浅灰） */
#define COLOR_TEXT_SECONDARY lv_color_hex(0xDDDDDD)

/** @brief 分隔线颜色 */
#define COLOR_SEPARATOR lv_color_hex(0xAAAAAA)

/** @brief 装饰点颜色 */
#define COLOR_ACCENT lv_color_hex(0x888888)

/* ── UI控件 ─────────────────────────────────────────────────── */

static lv_obj_t *s_time_label = NULL;
static lv_obj_t *s_date_label = NULL;
static lv_obj_t *s_separator_line = NULL;
static lv_obj_t *s_dot_left = NULL;
static lv_obj_t *s_dot_right = NULL;

/**
 * @brief 创建分隔线装饰
 */
static void create_separator(lv_obj_t *parent)
{
    /* 创建分隔线容器 */
    s_separator_line = lv_obj_create(parent);
    lv_obj_set_size(s_separator_line, 80, 1);
    lv_obj_align(s_separator_line, LV_ALIGN_CENTER, 0, SEPARATOR_Y_OFFSET);
    lv_obj_set_style_bg_color(s_separator_line, COLOR_SEPARATOR, LV_PART_MAIN);
    // lv_obj_set_style_bg_opa(s_separator_line, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_separator_line, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_separator_line, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_separator_line, 0, LV_PART_MAIN);

    /* 左侧装饰点 */
    s_dot_left = lv_obj_create(parent);
    lv_obj_set_size(s_dot_left, 3, 3);
    lv_obj_align(s_dot_left, LV_ALIGN_CENTER, -42, SEPARATOR_Y_OFFSET);
    lv_obj_set_style_bg_color(s_dot_left, COLOR_ACCENT, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_dot_left, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_dot_left, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_dot_left, 3, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_dot_left, 0, LV_PART_MAIN);

    /* 右侧装饰点 */
    s_dot_right = lv_obj_create(parent);
    lv_obj_set_size(s_dot_right, 3, 3);
    lv_obj_align(s_dot_right, LV_ALIGN_CENTER, 42, SEPARATOR_Y_OFFSET);
    lv_obj_set_style_bg_color(s_dot_right, COLOR_ACCENT, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_dot_right, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_dot_right, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_dot_right, 3, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_dot_right, 0, LV_PART_MAIN);
}

/**
 * @brief LVGL定时器回调函数 - 每秒更新时钟显示
 */
static void clock_update_timer_cb(lv_timer_t *timer)
{
    time_t now;
    struct tm time_info;
    time(&now);
    localtime_r(&now, &time_info);

    /* 格式化时间字符串 HH:MM:SS */
    char time_buf[16];
    strftime(time_buf, sizeof(time_buf), "%H:%M:%S", &time_info);
    lv_label_set_text(s_time_label, time_buf);

    /* 格式化日期字符串 YYYY-MM-DD 周X */
    char date_buf[32];
    const char *weekdays[] = {"周日", "周一", "周二", "周三", "周四", "周五", "周六"};
    snprintf(date_buf, sizeof(date_buf), "%04d-%02d-%02d %s",
             time_info.tm_year + 1900,
             time_info.tm_mon + 1,
             time_info.tm_mday,
             weekdays[time_info.tm_wday]);
    lv_label_set_text(s_date_label, date_buf);
}

void clock_ui_create(void)
{
    lv_obj_t *scr = lv_screen_active();

    /* ── 设置纯黑背景 ──────────────────────────────────────── */
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* ── 创建时间标签（大号，居中偏上） ────────────────────── */
    s_time_label = lv_label_create(scr);
    lv_obj_set_style_text_color(s_time_label, COLOR_TEXT_PRIMARY, LV_PART_MAIN);
    lv_obj_set_style_text_font(s_time_label, TIME_FONT, LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(s_time_label, 2, LV_PART_MAIN);
    lv_obj_align(s_time_label, LV_ALIGN_CENTER, 0, TIME_Y_OFFSET);

    /* ── 创建分隔线装饰 ────────────────────────────────────── */
    create_separator(scr);

    /* ── 创建日期标签（中号，分隔线下方） ──────────────────── */
    s_date_label = lv_label_create(scr);
    lv_obj_set_style_text_color(s_date_label, COLOR_TEXT_SECONDARY, LV_PART_MAIN);
    lv_obj_set_style_text_font(s_date_label, DATE_FONT, LV_PART_MAIN);
    lv_obj_align(s_date_label, LV_ALIGN_CENTER, 0, DATE_Y_OFFSET);

    /* ── 创建定时器并立即更新 ──────────────────────────────── */
    lv_timer_create(clock_update_timer_cb, 1000, NULL);
    clock_update_timer_cb(NULL);

    ESP_LOGI(TAG, "时钟UI创建完成（现代风格）");
}
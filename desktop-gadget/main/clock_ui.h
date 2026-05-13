/**
 * @file clock_ui.h
 * @brief 时钟界面UI模块
 *
 * 提供时钟界面创建和更新功能
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建时钟显示界面
 *
 * 在LVGL活动屏幕上创建时间、日期和状态标签，
 * 并创建定时器每秒更新显示内容。
 */
void clock_ui_create(void);

#ifdef __cplusplus
}
#endif
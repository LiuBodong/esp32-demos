/**
 * ESP32 BLE LED 控制器（增强版 - 支持呼吸与闪烁双模式）
 *
 * 功能：
 *   - 手机通过 BLE 连接后，发送 '1' 开启 LED 效果，发送 '0' 关闭
 *   - 发送 'R' 切换到呼吸模式（PWM 渐变）
 *   - 发送 'S' 切换到闪烁模式（固定频率开关）
 *   - 发送 'F' 查询当前模式（通过 Notify 返回）
 *   - ESP32 每 3 秒通过 Notify 向手机推送计数数据
 *   - 断开连接后自动重新广播
 *
 * 技术栈：
 *   - ESP-IDF + NimBLE（轻量级 BLE 协议栈）
 *   - LEDC（PWM 控制器）实现呼吸灯
 *   - FreeRTOS 多任务（LED 效果任务 + BLE TX 任务）
 */

#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"

#include "nvs_flash.h"

// ======= NimBLE 相关头文件 =======
#include "esp_nimble_hci.h"              // NimBLE HCI 接口，负责和蓝牙硬件通信
#include "nimble/nimble_port.h"          // NimBLE 协议栈核心，提供事件循环
#include "nimble/nimble_port_freertos.h" // 把 NimBLE 事件循环放到 FreeRTOS 任务中运行
#include "host/ble_hs.h"                 // BLE Host 层 API（GATT、GAP 的核心接口）
#include "services/gap/ble_svc_gap.h"    // GAP 服务（广播、连接管理）
#include "services/gatt/ble_svc_gatt.h"  // GATT 服务（属性表管理）

// ======= GPIO 与 PWM =======
#include "driver/gpio.h"
#include "driver/ledc.h"

// ======= Kconfig 配置 =======
#include "sdkconfig.h"

// =====================
//  常量和全局变量
// =====================

static const char *TAG = "main";

// 从 Kconfig 读取设备名称
static const char *DEVICE_NAME = CONFIG_APP_LED_BT_CONTROLLER_BT_NAME;

// 从 Kconfig 读取 LED 引脚号
#define LED_PIN CONFIG_APP_LED_BT_CONTROLLER_LED_PIN

// 从 Kconfig 读取 LED 点亮电平
// 如果选了 "Active Low"，则低电平(0) = LED 亮
// 如果选了 "Active High"，则高电平(1) = LED 亮
#ifdef CONFIG_APP_LED_BT_CONTROLLER_LED_ACTIVE_LOW
#define LED_ON_LEVEL 0  // 低电平点亮
#define LED_OFF_LEVEL 1 // 高电平熄灭
#else
#define LED_ON_LEVEL 1  // 高电平点亮
#define LED_OFF_LEVEL 0 // 低电平熄灭
#endif

// LED 模式枚举（新增）
typedef enum {
    LED_MODE_BREATH,  // 呼吸模式（PWM 渐变）
    LED_MODE_BLINK    // 闪烁模式（开关切换）
} led_mode_t;

// 当前 LED 模式（初始值从 Kconfig 读取，新选项）
#ifdef CONFIG_APP_LED_BT_CONTROLLER_DEFAULT_MODE_BREATH
static led_mode_t current_mode = LED_MODE_BREATH;
#else
static led_mode_t current_mode = LED_MODE_BLINK;
#endif

// LED 效果开关标志（volatile 表示可能被多个任务/中断访问，编译器不要优化它）
// 0 = 关闭效果，1 = 开启效果
static volatile int led_effect_enabled = 0;

// BLE 连接句柄，初始为"无连接"状态
// 连接成功后会被赋值，断开后重置，用来判断是否可以发送 Notify
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

// BLE 地址类型，由 NimBLE 协议栈自动推断
static uint8_t own_addr_type;

// TX 特征的值句柄
// NimBLE 给每个特征分配一个数字句柄，发送 Notify 时需要用这个句柄来指定发给哪个特征
static uint16_t tx_char_val_handle;

// =====================
//  PWM 相关常量（新增）
// =====================
#define LEDC_TIMER      LEDC_TIMER_0        // 使用定时器 0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE // 低速模式（ESP32-C3 只有低速）
#define LEDC_CHANNEL    LEDC_CHANNEL_0      // 使用通道 0
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT   // 13 位分辨率，范围 0~8191
#define LEDC_FREQ       5000                // 频率 5kHz，人眼完全看不出闪烁

// =====================
//  128-bit UUID 定义
// =====================
//
// BLE 标准的 128-bit UUID 格式：
//   xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
//
// Bluetooth SIG 规定的标准模板是：
//   0000xxxx-0000-1000-8000-00805f9b34fb
//
// 前 16 位（xxxx 部分）是自定义部分，后面是固定模板。
// 但 NimBLE 的 BLE_UUID128_INIT() 是按字节逆序填入的（小端序），
// 所以填的时候要把 UUID 倒过来写。
//
// 例如 Service UUID = 000000ff-0000-1000-8000-00805f9b34fb
// 完整 16 字节（从低到高）：
//   fb 34 9b 5f 80 00 00 80 00 10 00 00 ff 00 00 00
//                       ↑ 这里是自定义的 ff 00 00 00

// Service UUID:  000000ff-0000-1000-8000-00805f9b34fb
static const ble_uuid128_t gatt_svr_svc_uuid = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00);

// RX 特征 UUID: 0000ff01-0000-1000-8000-00805f9b34fb（手机 → ESP32）
static const ble_uuid128_t gatt_svr_chr_rx_uuid = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00);

// TX 特征 UUID: 0000ff02-0000-1000-8000-00805f9b34fb（ESP32 → 手机）
static const ble_uuid128_t gatt_svr_chr_tx_uuid = BLE_UUID128_INIT(
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x02, 0xff, 0x00, 0x00);

// =====================
//  前向声明
// =====================
// C 语言要求函数在使用前必须声明或定义
// 这些函数定义在后面，但前面要用到，所以先在这里声明一下

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_app_advertise(void);
static void led_effect_task(void *pvParameter);
static void led_set_duty(uint32_t duty);
static void led_set_raw(bool on);
static void stop_led_effect(void);

// =====================
//  GATT 服务表定义
// =====================
//
// 这个数组告诉 NimBLE：我有哪些服务，每个服务有哪些特征
// NimBLE 启动时会根据这个表自动注册 GATT 属性

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        // 第一个服务：自定义 LED 控制服务
        .type = BLE_GATT_SVC_TYPE_PRIMARY, // 主服务（不是次级服务）
        .uuid = &gatt_svr_svc_uuid.u,      // 服务的 UUID
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                // ---- RX 特征 ----
                // 手机通过写入这个特征来发送命令给 ESP32
                .uuid = &gatt_svr_chr_rx_uuid.u,  // 特征的 UUID
                .access_cb = gatt_svr_chr_access, // 读写时的回调函数
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                // WRITE: 需要响应的写入（可靠，但稍慢）
                // WRITE_NO_RSP: 不需要响应的写入（更快，适合频繁写入）
            },
            {
                // ---- TX 特征 ----
                // ESP32 通过 Notify 主动推送数据给手机
                .uuid = &gatt_svr_chr_tx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                // READ: 手机可以读取这个特征的值
                // NOTIFY: ESP32 可以主动推送数据给手机（手机需先订阅）
                .val_handle = &tx_char_val_handle,
                // val_handle: NimBLE 会把实际的句柄值写入这个变量
                // 后面发 Notify 时需要这个句柄来指定目标特征
            },
            {
                0, /* 特征列表结束标记 */
            },
        },
    },
    {
        0, /* 服务列表结束标记 */
    },
};

// =====================
//  LED 底层控制函数（新增 / 增强）
// =====================

/**
 * 设置 PWM 占空比（用于呼吸模式）
 * @param duty 占空比值，范围 0 ~ 8191（13 位分辨率）
 *
 * 为什么单独写这个函数？
 * 因为 LED 可能有 Active Low / Active High 的区别，
 * 这里统一把 "duty 越大 = 越亮" 的逻辑抽象出来，
 * 内部根据配置反转电平。
 */
static void led_set_duty(uint32_t duty)
{
    // 如果 LED 是低电平有效，我们需要反转占空比的意义
    // 例如 duty=8191（最大亮度） -> 对于 Active Low，应该输出低电平占空比最大
    #ifdef CONFIG_APP_LED_BT_CONTROLLER_LED_ACTIVE_LOW
        duty = 8191 - duty;
    #endif
    // 设置 PWM 占空比
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    // 必须调用 update 才能让设置立即生效
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

/**
 * 设置 LED 原始开关状态（用于闪烁模式）
 * @param on true = 点亮，false = 熄灭
 *
 * 【重大修复说明！！】
 * 这个函数原本是使用 gpio_set_level() 控制电平。
 * 但是！因为我们在 app_main 里把这个引脚交给了 PWM (LEDC) 管理，
 * 普通的 GPIO 控制对它就失效了！(这就是你切换闪烁模式没反应的罪魁祸首)。
 *
 * 现在的完美解法：复用 PWM 控制逻辑！
 * - on=true 时，直接让占空比为 8191 (最大)，等同于常亮！
 * - on=false 时，直接让占空比为 0 (最小)，等同于常灭！
 */
static void led_set_raw(bool on)
{
    // 修复冲突：复用上面写好的 led_set_duty 函数
    led_set_duty(on ? 8191 : 0);
}

/**
 * 停止 LED 效果并确保 LED 彻底关闭
 *
 * 这个函数可以从中断或别的任务调用
 * 因为 led_effect_enabled 是 volatile 的，读写是原子的
 */
static void stop_led_effect(void)
{
    led_effect_enabled = 0; // 告诉 led_effect_task 任务停止效果
    led_set_raw(false);     // 直接熄灭 LED（无论当前在什么模式，底层现在都会把 PWM 设为 0）
}

// =====================
//  LED 效果任务（FreeRTOS 任务）—— 大幅增强版
// =====================
//
// 这是一个独立的 FreeRTOS 任务，一直在后台运行
// 它检查 led_effect_enabled 标志和 current_mode 来决定执行哪种动画

static void led_effect_task(void *pvParameter)
{
    // ---- 呼吸模式相关变量 ----
    int duty = 0;               // 当前 PWM 占空比，范围 0~8191
    int step = 50;              // 每次变化的步长，步长越大呼吸越快
    bool increasing = true;     // 当前是渐亮还是渐暗

    // ---- 闪烁模式相关变量 ----
    bool blink_state = false;   // LED 当前开关状态

    // 从 Kconfig 读取呼吸速度（步进延时）和闪烁间隔
    const int breath_delay = CONFIG_APP_LED_BT_CONTROLLER_BREATH_SPEED_MS;
    const int blink_interval = CONFIG_APP_LED_BT_CONTROLLER_BLINK_INTERVAL_MS;

    while (1) // 任务永远循环
    {
        // 如果效果被禁用，就保持熄灭并等待
        if (!led_effect_enabled) {
            // 确保 LED 是熄灭的（既然底层都修复成统一复用 PWM 了，这里可以直接调 led_set_raw）
            led_set_raw(false);
            // 空闲时延迟 100ms，减少 CPU 占用
            vTaskDelay(pdMS_TO_TICKS(100));
            continue; // 跳过后面的效果逻辑
        }

        // ===== 效果已启用，根据当前模式执行动画 =====
        if (current_mode == LED_MODE_BREATH) {
            // ----- 呼吸模式：PWM 渐变 -----
            // 设置当前占空比
            led_set_duty(duty);

            // 更新占空比，实现渐亮渐暗
            if (increasing) {
                duty += step;
                if (duty >= 8191) {     // 达到最大亮度
                    duty = 8191;
                    increasing = false; // 改为渐暗
                }
            } else {
                duty -= step;
                if (duty <= 0) {        // 达到最小亮度（熄灭）
                    duty = 0;
                    increasing = true;  // 改为渐亮
                }
            }
            // 延时，控制呼吸速度
            vTaskDelay(pdMS_TO_TICKS(breath_delay));
        } else {
            // ----- 闪烁模式：开关切换 -----
            blink_state = !blink_state;   // 翻转状态
            led_set_raw(blink_state);     // 应用到 GPIO（此时底层已修复，正常闪烁）
            vTaskDelay(pdMS_TO_TICKS(blink_interval));
        }
    }
}

// =====================
//  GATT 读写回调函数（增强版，支持新指令）
// =====================
//
// 当手机读或写某个特征时，NimBLE 会调用这个函数
// 我们在这里处理具体的业务逻辑

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // 根据操作类型分别处理
    switch (ctxt->op)
    {
    // ---- 写入操作（手机发数据给 ESP32）----
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (ctxt->om->om_len > 0)
        {
            // ctxt->om 是 NimBLE 的内存块结构（os_mbuf）
            // om_data 指向实际数据，om_len 是数据长度
            uint8_t cmd = ctxt->om->om_data[0];

            // 解析指令（新增了 R、S、F 指令）
            switch (cmd) {
                case '1':
                case 0x01:
                    // 收到 '1'（字符）或 0x01（字节），开启 LED 效果
                    ESP_LOGI(TAG, "收到指令: 开启 LED 效果");
                    led_effect_enabled = 1;
                    break;

                case '0':
                case 0x00:
                    // 收到 '0'（字符）或 0x00（字节），关闭 LED 效果
                    ESP_LOGI(TAG, "收到指令: 关闭 LED 效果");
                    stop_led_effect();
                    break;

                case 'R':
                case 'r':
                    // 切换到呼吸模式
                    ESP_LOGI(TAG, "收到指令: 切换到呼吸模式");
                    current_mode = LED_MODE_BREATH;
                    // 如果当前效果是开启的，模式切换后立刻生效
                    break;

                case 'S':
                case 's':
                    // 切换到闪烁模式
                    ESP_LOGI(TAG, "收到指令: 切换到闪烁模式");
                    current_mode = LED_MODE_BLINK;
                    break;

                case 'F':
                case 'f':
                    // 查询当前模式，通过 Notify 返回
                    ESP_LOGI(TAG, "收到指令: 查询当前模式");
                    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                        const char *mode_str = (current_mode == LED_MODE_BREATH) ?
                                               "Mode: Breath" : "Mode: Blink";
                        // 把字符串打包成 os_mbuf 发送
                        struct os_mbuf *om = ble_hs_mbuf_from_flat(mode_str, strlen(mode_str));
                        if (om) {
                            ble_gatts_notify_custom(conn_handle, tx_char_val_handle, om);
                        }
                    }
                    break;

                default:
                    // 其他数据，打印十六进制方便调试
                    ESP_LOGW(TAG, "收到未知指令: 0x%02X", cmd);
                    break;
            }
        }
        return 0; // 返回 0 表示处理成功

    // ---- 读取操作（手机读取 ESP32 的数据）----
    case BLE_GATT_ACCESS_OP_READ_CHR:
    {
        // 返回一段提示信息，告诉手机支持哪些指令
        const char *resp = "ESP32 Ready (1=on,0=off,R=Breath,S=Blink,F=Query)";
        // os_mbuf_append 把数据追加到响应缓冲区
        int rc = os_mbuf_append(ctxt->om, resp, strlen(resp));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    default:
        // 未知操作，返回错误
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// =====================
//  TX 定时 Notify 任务
// =====================
//
// 每 3 秒检查一次：如果有设备连接，就发送一条 Notify 数据
// 这模拟了传感器数据推送的场景

static void tx_task(void *pvParameter)
{
    char notify_data[32];
    int count = 0;

    while (1)
    {
        // 每 3 秒执行一次
        vTaskDelay(pdMS_TO_TICKS(3000));

        // 只有在有设备连接时才发送
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            // 构造要发送的数据
            snprintf(notify_data, sizeof(notify_data), "Data Count: %d", count++);

            // 把普通内存数据打包成 NimBLE 的 os_mbuf 格式
            // os_mbuf 是 NimBLE 内部使用的链式内存块，BLE 协议栈需要这种格式
            struct os_mbuf *om = ble_hs_mbuf_from_flat(notify_data, strlen(notify_data));
            if (om == NULL)
            {
                ESP_LOGE(TAG, "mbuf 分配失败");
                continue;
            }

            // 发送 Notify
            // conn_handle: 发给哪个连接的设备
            // tx_char_val_handle: 从哪个特征发送（就是我们之前记录的那个句柄）
            // om: 要发送的数据
            int rc = ble_gatts_notify_custom(conn_handle, tx_char_val_handle, om);
            if (rc == 0)
            {
                ESP_LOGI(TAG, "已发送 Notify: %s", notify_data);
            }
            else
            {
                ESP_LOGE(TAG, "发送失败, 错误码: %d", rc);
            }
        }
    }
}

// =====================
//  GAP 事件回调
// =====================
//
// GAP（Generic Access Profile）负责广播、连接、配颠等
// 当这些事件发生时，NimBLE 会调用这个函数

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // ---- 连接事件 ----
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            // 连接成功（status == 0 表示无错误）
            ESP_LOGI(TAG, "设备已连接! conn_handle=%d", event->connect.conn_handle);
            conn_handle = event->connect.conn_handle; // 保存连接句柄，后续 Notify 要用
        }
        else
        {
            // 连接失败，重新开始广播，让手机可以再次搜索到
            ESP_LOGI(TAG, "连接失败，重启广播...");
            ble_app_advertise();
        }
        break;

    // ---- 断开事件 ----
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "设备已断开连接，重启广播...");
        conn_handle = BLE_HS_CONN_HANDLE_NONE; // 重置为"无连接"
        ble_app_advertise();                   // 必须重新广播，否则手机找不到
        break;

    // ---- 订阅事件 ----
    // 手机开启或关闭 Notify 订阅时触发
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "手机修改了订阅状态 (Notify: %d)", event->subscribe.cur_notify);
        // cur_notify: 1 = 手机订阅了 Notify，0 = 取消订阅
        break;
    }
    return 0;
}

// =====================
//  开启 BLE 广播
// =====================
//
// 广播 = 周期性地向外发射信号，让周围的手机能发现你
// 没有广播 = 手机搜不到你

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params; // 广播参数
    struct ble_hs_adv_fields fields;      // 广播数据内容

    // ---- 配置广播数据 ----
    memset(&fields, 0, sizeof(fields));

    // flags: 告诉扫描设备一些基本信息
    // BLE_HS_ADV_F_DISC_GEN: 开启通用发现模式（谁都能发现我）
    // BLE_HS_ADV_F_BREDR_UNSUP: 不支持经典蓝牙（BR/EDR），只支持 BLE
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // 设备名称：让手机扫描时能看到名字
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1; // 表示这是完整名称（不是截断的）

    // 把数据写入广播载荷
    ble_gap_adv_set_fields(&fields);

    // ---- 配置广播参数 ----
    memset(&adv_params, 0, sizeof(adv_params));

    // conn_mode: 连接模式
    // BLE_GAP_CONN_MODE_UND: 无定向连接（任何人都可以连我）
    // 另外还有定向连接模式（只允许指定设备连接）
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;

    // disc_mode: 发现模式
    // BLE_GAP_DISC_MODE_GEN: 通用可发现（任何扫描设备都能看到）
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // 启动广播
    // own_addr_type: 地址类型（公共地址或随机地址）
    // NULL: 不指定目标地址（无定向广播）
    // BLE_HS_FOREVER: 永久广播（直到手动停止或被连接）
    // ble_gap_event: 广播期间发生的事件（如连接请求）会回调这个函数
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                      ble_gap_event, NULL);

    ESP_LOGI(TAG, "正在广播...");
}

// =====================
//  NimBLE 同步回调
// =====================
//
// 当 NimBLE 协议栈初始化完成并同步后，会自动调用这个函数
// 这是开始广播的最佳时机（协议栈还没准备好就广播会失败）

static void ble_app_on_sync(void)
{
    // 自动推断地址类型
    // 参数 0 表示使用默认的地址类型偏好
    // 结果会写入 own_addr_type
    ble_hs_id_infer_auto(0, &own_addr_type);

    // 协议栈就绪，开始广播
    ble_app_advertise();
}

// =====================
//  NimBLE Host 任务
// =====================
//
// 这个任务运行 NimBLE 的事件循环
// 所有 BLE 事件（连接、断开、读写、Notify 等）都在这里处理
// nimble_port_run() 是阻塞的，会一直运行直到调用 nimble_port_stop()

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host 任务启动");
    nimble_port_run();             // 阻塞运行 NimBLE 事件循环
    nimble_port_freertos_deinit(); // 理论上不会执行到这里
}

// =====================
//  NVS 初始化
// =====================
//
// NVS（Non-Volatile Storage）是 ESP32 的非易失性存储
// BLE 协议栈需要 NVS 来保存配对信息、绑定信息等
// 如果 NVS 损坏或版本不匹配，需要擦除后重新初始化

static esp_err_t app_nvs_flash_init(void)
{
    esp_err_t ret = nvs_flash_init();

    // 检查常见的初始化失败原因
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS 分区满了或者版本不匹配，擦除后重试
        ESP_LOGW(TAG, "NVS flash init failed, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret); // 如果还是失败，直接报错重启
    return ret;
}

// =====================
//  主入口
// =====================
//
// app_main() 是 ESP-IDF 程序的入口函数
// ESP-IDF 不用标准的 main()，而是用 app_main()
// 它在一个特殊的 FreeRTOS 任务中运行

void app_main(void)
{
    // =========================================
    //  第一步：初始化 LEDC PWM 硬件
    // =========================================
    // 为什么最先做？
    // 因为 ESP32 上电时引脚状态不可控，我们尽早将它配置好，避免乱闪。
    //
    // 【重要修复】：把之前的常规 GPIO 配置(gpio_config)删掉了！
    // 既然要跑 PWM 效果，就必须完全把它交给 LEDC 控制器。
    // 一旦给它上了 PWM 的“紧箍咒”，普通 GPIO 逻辑就彻底失效，所以我们直接在这里初始化它，
    // 以后的所有动作（包括常亮、常灭），统统通过调 PWM 占空比来解决！

    // 1. 配置 PWM 定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,           // 低速模式
        .timer_num       = LEDC_TIMER,          // 定时器编号 0
        .duty_resolution = LEDC_DUTY_RES,       // 13 位分辨率（0~8191）
        .freq_hz         = LEDC_FREQ,           // 5kHz，人眼无闪烁
        .clk_cfg         = LEDC_AUTO_CLK        // 自动选择时钟源
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. 配置 PWM 通道（将定时器输出绑定到指定 GPIO）
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,                // 必须与定时器一致
        .channel    = LEDC_CHANNEL,             // 通道 0
        .timer_sel  = LEDC_TIMER,               // 使用上面配置的定时器 0
        .intr_type  = LEDC_INTR_DISABLE,        // 不使用中断
        .gpio_num   = LED_PIN,                  // 绑定到 LED 引脚
        
        // 关键修复点：为了防止单片机刚上电时灯乱闪一下，我们在这里给个初始熄灭的占空比
        #ifdef CONFIG_APP_LED_BT_CONTROLLER_LED_ACTIVE_LOW
        .duty       = 8191,                     // 低电平亮，所以给它最大占空比(高电平)让它先灭着
        #else
        .duty       = 0,                        // 高电平亮，给它 0 占空比(低电平)让它灭着
        #endif
        
        .hpoint     = 0                         // 不使用相位偏移
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // 确保软状态也是关闭的（底层自动会处理好 PWM）
    led_set_raw(false); 

    // =========================================
    //  第二步：初始化 BLE
    // =========================================

    // 2a. 初始化 NVS（BLE 需要它存储配对信息等）
    ESP_ERROR_CHECK(app_nvs_flash_init());

    // 2b. 初始化 NimBLE 协议栈底层（HCI 层、内存池等）
    ESP_ERROR_CHECK(nimble_port_init());

    // 2c. 设置 GAP 设备名称（手机扫描时看到的名字）
    ble_svc_gap_device_name_set(DEVICE_NAME);

    // 2d. 注册 GATT 服务
    //    ble_gatts_count_cfg: 告诉 NimBLE 我有多少服务/特征（用于分配内存）
    //    ble_gatts_add_svcs:  把服务表注册到 GATT 服务器
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    // 2e. 设置同步回调
    //    当 NimBLE 协议栈完全初始化完成后，会调用 ble_app_on_sync()
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // =========================================
    //  第三步：启动所有任务
    // =========================================

    // 3a. 启动 NimBLE Host 任务
    //    nimble_port_freertos_init 会创建一个 FreeRTOS 任务来运行 BLE 事件循环
    //    这个任务会自动处理连接、读写、Notify 等所有 BLE 事件
    nimble_port_freertos_init(ble_host_task);

    // 3b. 启动 LED 效果任务（增强版，支持呼吸与闪烁）
    //    参数说明：
    //    - led_effect_task: 任务函数
    //    - "led_effect": 任务名称（调试时可以看到）
    //    - 2048: 栈大小（单位：字）
    //    - NULL: 传递给任务的参数
    //    - 5: 任务优先级（数字越大优先级越高）
    //    - NULL: 任务句柄（不需要获取）
    xTaskCreate(led_effect_task, "led_effect", 2048, NULL, 5, NULL);

    // 3c. 启动 TX 定时 Notify 任务
    xTaskCreate(tx_task, "tx_task", 2048, NULL, 5, NULL);

    // 打印初始化完成信息
    ESP_LOGI(TAG, "系统初始化完成。默认模式: %s",
             current_mode == LED_MODE_BREATH ? "呼吸" : "闪烁");

    // app_main 到这里就结束了
    // 但上面启动的 3 个任务会在后台一直运行
    // - ble_host_task:   处理 BLE 事件
    // - led_effect_task: 控制 LED 呼吸/闪烁效果
    // - tx_task:         定时发送 Notify
}
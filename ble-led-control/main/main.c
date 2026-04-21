#include <stdlib.h>
#include "esp_err.h"
#include "esp_log.h"

#include "nvs_flash.h"

// ======= Nimble 相关头文件 =======
#include "esp_nimble_hci.h"              // NimBLE HCI 接口
#include "nimble/nimble_port.h"          // NimBLE 协议栈核心
#include "nimble/nimble_port_freertos.h" // FreeRTOS 适配层
#include "host/ble_hs.h"                 // BLE Host 层 API
#include "services/gap/ble_svc_gap.h"    // GAP 服务（广播、连接管理）
#include "services/gatt/ble_svc_gatt.h"  // GATT 服务（属性表管理）

#include "driver/gpio.h" // 用于控制 LED 灯

// ======= Config ========
#include "sdkconfig.h"

static const char *TAG = "main";

static const char *DEVICE_NAME = CONFIG_APP_LED_BT_CONTROLLER_BT_NAME;

// 全局变量保存连接句柄和地址类型
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t own_addr_type;
static uint16_t tx_char_val_handle; // 记录 TX 特征的句柄，用于发送 Notify

// 定义自定义的 服务(Service) 和 特征(Characteristic) UUID
// 这里使用 16-bit UUID 作为简化的示例 (正式项目常用 128-bit)
static const ble_uuid16_t gatt_svr_svc_uuid = BLE_UUID16_INIT(0x00FF);    // 自定义服务 UUID
static const ble_uuid16_t gatt_svr_chr_rx_uuid = BLE_UUID16_INIT(0xFF01); // RX (ESP32接收)
static const ble_uuid16_t gatt_svr_chr_tx_uuid = BLE_UUID16_INIT(0xFF02); // TX (ESP32发送)

static uint16_t tx_char_val_handle; // 记录 TX 特征的句柄，用于发送 Notify

static volatile int led_blink_status = 0; // LED 状态变量，0: 关, 1: 开

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_app_advertise(void);
static void blink_led(void *pvParameter);
static void stop_blink_led(void);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                // RX 特征：允许手机写入数据
                .uuid = &gatt_svr_chr_rx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                // TX 特征：允许手机读取，且 ESP32 可主动 Notify
                .uuid = &gatt_svr_chr_tx_uuid.u,
                .access_cb = gatt_svr_chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tx_char_val_handle, // 绑定句柄，用于 Notify 发送
            },
            {
                0, /* 结束符 */
            }},
    },
    {
        0, /* 结束符 */
    },
};

static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        // 处理手机发来的写入请求
        if (ctxt->om->om_len > 0)
        {
            // ctxt->om 是一个 mbuf (内存块)，om_data 存放实际数据
            if (ctxt->om->om_len > 0)
            {
                if (ctxt->om->om_data[0] == '1' || ctxt->om->om_data[0] == 1)
                {
                    ESP_LOGI(TAG, "收到指令: 开启 LED 闪烁");
                    led_blink_status = 1; // 开启 LED 闪烁
                }
                else if (ctxt->om->om_data[0] == '0' || ctxt->om->om_data[0] == 0)
                {
                    ESP_LOGI(TAG, "收到指令: 关闭 LED 闪烁");
                    stop_blink_led(); // 关闭 LED 闪烁
                }
                else
                {
                    ESP_LOGW(TAG, "收到未知指令: %s", ctxt->om->om_data);
                }
            }
        }
        return 0;

    case BLE_GATT_ACCESS_OP_READ_CHR:
        // 处理手机发来的读取请求
        const char *resp = "ESP32 Ready";
        int rc = os_mbuf_append(ctxt->om, resp, strlen(resp));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static void blink_led(void *pvParameter)
{

    gpio_reset_pin(GPIO_NUM_8); // 重置 GPIO8，确保它处于已知状态
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << GPIO_NUM_8),  // 配置 GPIO8
        .mode = GPIO_MODE_OUTPUT,              // 设置为输出模式
        .pull_up_en = GPIO_PULLUP_DISABLE,     // 禁止上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁止下拉
        .intr_type = GPIO_INTR_DISABLE,        // 禁止中断
    };
    gpio_config(&config); // 应用 GPIO 配置
    static bool led_on = false;

    while (1)
    {
        if (led_blink_status == 0)
        {
            if (led_on)
            {
                gpio_set_level(GPIO_NUM_8, 1); // 确保 LED 灯关闭
                led_on = false;
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // 如果不需要闪烁，稍微延迟一下，减少 CPU 占用
            continue;
        }
        gpio_set_level(GPIO_NUM_8, led_on); // 控制内置 LED 灯
        led_on = !led_on;                   // 切换状态
        vTaskDelay(pdMS_TO_TICKS(500));     // 延迟 500 ms
    }
}

static void stop_blink_led(void)
{
    led_blink_status = 0;          // 停止 LED 闪烁
    gpio_set_level(GPIO_NUM_8, 1); // 确保 LED 灯关闭
}

void tx_task(void *pvParameter)
{
    char notify_data[32];
    int count = 0;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(3000)); // 每 3 秒执行一次

        // 如果当前有设备连接，才进行数据发送
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE)
        {
            snprintf(notify_data, sizeof(notify_data), "Data Count: %d", count++);

            // 将平坦的数据字符串打包进 NimBLE 特有的 os_mbuf 结构中
            struct os_mbuf *om = ble_hs_mbuf_from_flat(notify_data, strlen(notify_data));

            // 发送 Notify 到手机
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

// GAP 事件处理回调 (连接/断开/广播)
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            ESP_LOGI(TAG, "设备已连接!");
            conn_handle = event->connect.conn_handle; // 保存连接句柄
        }
        else
        {
            ESP_LOGI(TAG, "连接失败，重启广播...");
            ble_app_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "设备已断开连接，重启广播...");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_app_advertise(); // 断开后必须重新开启广播才能再次被发现
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "手机修改了订阅状态 (Notify: %d)", event->subscribe.cur_notify);
        break;
    }
    return 0;
}

// 开启蓝牙广播
static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;

    // 配置广播数据载荷 (Advertisement Data)
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP; // 开启发现模式，不支持传统蓝牙
    fields.name = (uint8_t *)ble_svc_gap_device_name();              // 取 GAP 名称
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields); // 应用载荷

    // 配置并启动广播参数
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // 无定向连接模式 (所有人都能连)
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // 普通发现模式

    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    ESP_LOGI(TAG, "正在广播...");
}

// NimBLE 启动同步回调
static void ble_app_on_sync(void)
{
    // 协议栈同步成功后，自动获取地址类型，并开启广播
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_app_advertise();
}

// NimBLE Host 任务
void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host 任务启动");
    nimble_port_run(); // 阻塞运行 NimBLE 事件循环
    nimble_port_freertos_deinit();
}

// =====================
//  初始化 nvs_flash
// =====================

esp_err_t app_nvs_flash_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS flash init failed, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    return ret;
}

void app_main(void)
{
    xTaskCreate(blink_led, "blink_led", 2048, NULL, 5, NULL); // 启动 LED 闪烁任务
    // 初始化 nvs_flash
    ESP_ERROR_CHECK(app_nvs_flash_init());
    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
    xTaskCreate(tx_task, "tx_task", 2048, NULL, 5, NULL);
}
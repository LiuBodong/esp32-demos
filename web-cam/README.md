# ESP32 Web Cam

基于 ESP-IDF 的 Wi-Fi 网络摄像头项目，支持 MJPEG 实时视频流和单帧抓拍，兼容 ESP32-CAM 和 ESP32-S3-CAM 开发板。

## 功能特性

- **MJPEG 实时流**：浏览器直接访问 `/stream` 观看实时画面
- **单帧抓拍**：通过 `/capture` 获取当前帧 JPEG 图片
- **多板子支持**：ESP32-CAM (AI-Thinker) / ESP32-S3-CAM (Freenove) 自动引脚适配
- **多传感器适配**：OV2640 / OV3660 / OV5640 自动检测与专属参数优化
- **多分辨率**：QVGA ~ 5MP 可选，OV5640 支持 2560x1440 及 2592x1944
- **静态 IP / DHCP**：可选静态 IP 配置，方便固定访问地址
- **Web 管理页面**：内置暗色主题 HTML 页面，移动端适配

## 硬件要求

| 开发板                  | 推荐 Sensor     | PSRAM | 备注                             |
| ----------------------- | --------------- | ----- | -------------------------------- |
| ESP32-CAM (AI-Thinker)  | OV2640          | 4MB   | 高分辨率时帧缓冲降为 1，可能丢帧 |
| ESP32-S3-CAM (Freenove) | OV3660 / OV5640 | 8MB+  | 推荐使用，双缓冲流畅             |

## 快速开始

### 1. 环境准备

确保已安装 [ESP-IDF](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/get-started/) (>= v4.1)。

### 2. 配置项目

```bash
idf.py menuconfig
```

进入 `App WebCam Configuration` 菜单：

#### Camera Board Type

选择开发板类型：
- **ESP32-CAM (AI-Thinker)** — 经典 ESP32-CAM
- **ESP32-S3-CAM (Freenove / Standard)** — ESP32-S3-CAM

#### Camera 子菜单

| 配置项                 | 默认值        | 说明                                |
| ---------------------- | ------------- | ----------------------------------- |
| Camera Sensor          | OV3660        | 摄像头传感器型号                    |
| Camera Frame Size      | VGA (640x480) | 分辨率，OV5640 可选 QHD/5MP         |
| Camera Max FPS         | 10            | 最大帧率 (1~25)                     |
| JPEG Quality           | 12            | JPEG 质量，数值越小画质越好 (10~63) |
| Camera XCLK Freq (MHz) | 10            | 传感器时钟频率 (1~40)               |

#### Wifi 子菜单

| 配置项          | 默认值        | 说明            |
| --------------- | ------------- | --------------- |
| Wifi SSID       | my_ssid       | Wi-Fi 名称      |
| Wifi Password   | *(空)*        | Wi-Fi 密码      |
| Use Static IP   | y             | 是否使用静态 IP |
| Static IP       | 192.168.1.201 | 静态 IP 地址    |
| Gateway Address | 192.168.1.1   | 网关            |
| Netmask         | 255.255.255.0 | 子网掩码        |
| DNS             | 192.168.1.1   | DNS 服务器      |

#### Httpd 子菜单

| 配置项            | 默认值 | 说明          |
| ----------------- | ------ | ------------- |
| Httpd Server Port | 80     | HTTP 服务端口 |

### 3. 编译 & 烧录

```bash
idf.py build
idf.py -p <PORT> flash monitor
```

### 4. 访问

烧录成功后，串口日志会打印访问地址：

```
I (xxxx) app_webcam: Please Visite: http://192.168.1.201:80
```

## HTTP 接口

| 路径       | 方法 | 说明                                    |
| ---------- | ---- | --------------------------------------- |
| `/`        | GET  | Web 管理页面（暗色主题，内嵌 MJPEG 流） |
| `/stream`  | GET  | MJPEG 实时视频流                        |
| `/capture` | GET  | 单帧 JPEG 抓拍，返回 `capture.jpg`      |

### 示例

- 浏览器直接访问：`http://192.168.1.201/`
- 获取单帧图片：`http://192.168.1.201/capture`
- 嵌入其他网页：`<img src="http://192.168.1.201/stream">`

## 项目结构

```
web-cam/
├── main/
│   ├── main.c            # 入口，依次初始化 Wi-Fi → Camera → HTTPD
│   ├── app_wifi.c/h      # Wi-Fi STA 连接、静态 IP / DHCP、自动重连
│   ├── app_camera.c/h    # 摄像头初始化、引脚适配、传感器专属调优
│   ├── app_httpd.c/h     # HTTP 服务器、MJPEG 流 & 抓拍接口
│   ├── Kconfig.projbuild # menuconfig 配置项定义
│   ├── CMakeLists.txt    # 组件构建脚本
│   └── idf_component.yml # 组件依赖 (esp32-camera)
├── CMakeLists.txt        # 项目构建脚本
└── sdkconfig             # 编译配置
```

## 架构流程

```
app_main()
  ├── nvs_flash_init()         # 初始化 NVS
  ├── app_wifi_init()          # 连接 Wi-Fi (STA 模式)
  ├── app_camera_init()        # 初始化摄像头 + 传感器调优
  └── app_httpd_init()         # 启动 HTTP 服务
        ├── GET /              → index_handler   (Web 页面)
        ├── GET /stream        → stream_handler  (MJPEG 流)
        └── GET /capture       → capture_handler (单帧抓拍)
```

## 传感器优化说明

代码针对不同传感器做了差异化调优：

- **OV2640**：放宽 AGC 增益天花板，开启 DSP AEC 补偿低光，降低饱和度修正暖色偏
- **OV3660**：适中 AGC 增益，自动白平衡校正偏青，启用坏点校正，默认垂直翻转
- **OV5640**：低 AGC 天花板利用大动态范围，高分辨率启用降噪和镜头校正

## 注意事项

- ESP32-CAM 内存有限，UXGA 及以上分辨率建议选用 ESP32-S3-CAM
- JPEG Quality 越小画质越好，但帧率会降低
- XCLK 频率过高可能导致传感器不稳定，建议 10~20MHz
- Wi-Fi 断开后会自动延迟 3 秒重连

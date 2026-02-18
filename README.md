# ESP32 HUB75 LED 矩阵屏图片显示系统

通过 WiFi 上传图片，在 HUB75 LED 矩阵屏上显示。

## 硬件参数

| 参数 | 值 |
|------|------|
| 单块屏分辨率 | 64×32 |
| 串联数量 | 2 块 |
| 总分辨率 | 128×32 |
| 扫描方式 | 1/16 扫 |
| 接口 | HUB75 |
| 色深 | 6 位 BCM (262144 色) |
| 驱动方式 | 手动位操作 (无 DMA) |
| 主控 | ESP32-D0WD-V3 双核 |

## 引脚定义

| 信号 | GPIO | 说明 |
|------|------|------|
| R1 | 27 | 上半屏红色 (已校正) |
| G1 | 25 | 上半屏绿色 (已校正) |
| B1 | 26 | 上半屏蓝色 (已校正) |
| R2 | 13 | 下半屏红色 (已校正) |
| G2 | 14 | 下半屏绿色 (已校正) |
| B2 | 17 | 下半屏蓝色 (已校正) |
| A | 32 | 行地址 bit0 |
| B | 33 | 行地址 bit1 |
| C | 21 | 行地址 bit2 |
| D | 15 | 行地址 bit3 |
| LAT | 4 | 锁存 |
| OE | 22 | 输出使能 |
| CLK | 16 | 时钟 |

> 注意：面板内部 RGB 接线与 HUB75 丝印标注不同，引脚已根据实测校正。

## 功能特性

- **手动 HUB75 驱动**：直接寄存器操作，避免 DMA 高速花屏
- **双核架构**：Core 1 独占 LED 扫描，Core 0 运行 WiFi + Web 服务器
- **Gamma 校正**：γ=2.5 查找表，改善 LED 上的图片显示效果
- **亮度控制**：通过 `BRIGHTNESS` 宏调节全局亮度 (0-255)
- **WiFi 自动重连**：断线自动恢复
- **mDNS**：支持 `http://esp32-led.local` 访问
- **开机自检**：依次全屏红→绿→蓝→白，每色 0.5 秒
- **中文串口日志**：所有日志带 `【模块】` 前缀

## API 接口

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/` | 状态页面 |
| GET | `/status` | JSON 状态信息 |
| POST | `/upload` | 上传图片 (Body: RGB888 Base64) |
| POST | `/clear` | 清空屏幕 |

### 上传格式

- 将 128×32 PNG 图片提取 RGB888 原始数据 (12288 字节)
- Base64 编码后作为 POST Body 发送到 `/upload`
- Content-Type: `text/plain`

## 编译与烧录

```bash
# 使用 PlatformIO CLI
~/.platformio/penv/bin/pio run --target upload

# 查看串口日志
~/.platformio/penv/bin/pio device monitor
```

## 配置

在 `src/main.cpp` 中修改：

```cpp
// WiFi
const char* ssid     = "你的WiFi名称";
const char* password = "你的WiFi密码";

// 亮度 (0-255)
#define BRIGHTNESS 30
```

## 配套前端

见 [ESP32-HUB75-LED-WebUI](../ESP32-HUB75-LED-WebUI) 项目。

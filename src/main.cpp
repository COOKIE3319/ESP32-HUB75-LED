/*
 * ESP32 HUB75 LED矩阵屏图片显示系统
 * =====================================
 * 分辨率: 128x32 (2块64x32串联)
 * 扫描: 1/16
 * 驱动: 手动位操作 (无DMA，避免花屏)
 * 色深: 6位BCM (262144色)
 * 功能: WiFi接收图片并显示
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "mbedtls/base64.h"
#include "soc/gpio_reg.h"
#include "esp_task_wdt.h"

// ==================== 引脚定义 ====================
// 注意: 面板内部RGB接线与HUB75标注不同，已根据实测校正
// 实测: GPIO25→绿, GPIO26→蓝, GPIO27→红 (HUB75标注R1/G1/B1)
#define R1_PIN  27
#define G1_PIN  25
#define B1_PIN  26
#define R2_PIN  13
#define G2_PIN  14
#define B2_PIN  17
#define A_PIN   32
#define B_PIN   33
#define C_PIN   21
#define D_PIN   15
#define LAT_PIN 4
#define OE_PIN  22
#define CLK_PIN 16

// ==================== 显示参数 ====================
#define PANEL_WIDTH    64
#define PANEL_HEIGHT   32
#define NUM_PANELS     2
#define DISPLAY_WIDTH  (PANEL_WIDTH * NUM_PANELS)  // 128
#define DISPLAY_HEIGHT PANEL_HEIGHT                  // 32
#define SCAN_ROWS      16                            // 1/16扫描
#define COLOR_DEPTH    6                             // 每通道6位色深 (BCM)
#define BRIGHTNESS     127                            // 全局亮度 (0-255)，30≈12%亮度

// ==================== WiFi 配置 ====================
const char* ssid     = "CMCC-Apfb";
const char* password = "d4mk9AE2";

// ==================== 全局变量 ====================
uint8_t frameBuffer[DISPLAY_WIDTH * DISPLAY_HEIGHT * 3] = {0};

WebServer server(80);

// Gamma校正表 (gamma=2.5，改善LED上的图片显示效果)
uint8_t gammaTable[256];

// GPIO 位掩码 (所有数据引脚均在GPIO 0-31范围)
const uint32_t R1_MASK  = (1UL << R1_PIN);
const uint32_t G1_MASK  = (1UL << G1_PIN);
const uint32_t B1_MASK  = (1UL << B1_PIN);
const uint32_t R2_MASK  = (1UL << R2_PIN);
const uint32_t G2_MASK  = (1UL << G2_PIN);
const uint32_t B2_MASK  = (1UL << B2_PIN);
const uint32_t DATA_MASK = R1_MASK | G1_MASK | B1_MASK | R2_MASK | G2_MASK | B2_MASK;
const uint32_t CLK_MASK  = (1UL << CLK_PIN);
const uint32_t LAT_MASK  = (1UL << LAT_PIN);
const uint32_t OE_MASK   = (1UL << OE_PIN);

// A(32), B(33) 高位GPIO寄存器掩码
const uint32_t A_HI_MASK = (1UL << (A_PIN - 32));
const uint32_t B_HI_MASK = (1UL << (B_PIN - 32));
const uint32_t C_MASK    = (1UL << C_PIN);
const uint32_t D_MASK    = (1UL << D_PIN);

// ==================== Gamma校正初始化 ====================
void initGammaTable() {
    float gamma = 2.5f;
    for (int i = 0; i < 256; i++) {
        // Gamma校正后再乘以亮度系数，有效限制最大输出值
        float val = powf((float)i / 255.0f, gamma) * (float)BRIGHTNESS + 0.5f;
        gammaTable[i] = (uint8_t)(val > 255.0f ? 255 : val);
    }
    Serial.printf("【初始化】Gamma校正表已生成 (γ=2.5, 亮度=%d/255≈%d%%)\n", BRIGHTNESS, BRIGHTNESS * 100 / 255);
}

// ==================== HUB75 引脚初始化 ====================
void initHUB75Pins() {
    Serial.println("【初始化】配置HUB75引脚...");

    const int pins[] = {R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN,
                        A_PIN, B_PIN, C_PIN, D_PIN, LAT_PIN, OE_PIN, CLK_PIN};
    for (int pin : pins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    digitalWrite(OE_PIN, HIGH); // 初始禁用输出

    Serial.println("【初始化】HUB75引脚配置完成");
}

// ==================== 行地址设置 (直接寄存器操作) ====================
inline void setRowAddress(uint8_t row) {
    // A(32) B(33) → 高位寄存器
    if (row & 0x01) REG_WRITE(GPIO_OUT1_W1TS_REG, A_HI_MASK);
    else            REG_WRITE(GPIO_OUT1_W1TC_REG, A_HI_MASK);
    if (row & 0x02) REG_WRITE(GPIO_OUT1_W1TS_REG, B_HI_MASK);
    else            REG_WRITE(GPIO_OUT1_W1TC_REG, B_HI_MASK);
    // C(21) D(15) → 低位寄存器
    if (row & 0x04) REG_WRITE(GPIO_OUT_W1TS_REG, C_MASK);
    else            REG_WRITE(GPIO_OUT_W1TC_REG, C_MASK);
    if (row & 0x08) REG_WRITE(GPIO_OUT_W1TS_REG, D_MASK);
    else            REG_WRITE(GPIO_OUT_W1TC_REG, D_MASK);
}

// ==================== LED显示扫描任务 (Core 1 独占) ====================
void displayScanTask(void *param) {
    // 将此任务从看门狗监控中移除，防止因扫描占用CPU导致WDT重启
    esp_task_wdt_delete(NULL);
    Serial.println("【显示】LED扫描任务已在Core 1启动 (独占核心, 已排除WDT监控)");

    const uint16_t baseDelay = 1; // 最低位延时(微秒)，值越大亮度越高但刷新率降低

    // 初始启用输出 (OE LOW)
    REG_WRITE(GPIO_OUT_W1TC_REG, OE_MASK);

    // ====================================================================
    // 优化扫描: 利用 HUB75 移位寄存器与输出锁存器独立的特性
    // 在上一行/位仍在显示 (OE LOW) 时，将下一行/位的数据移入移位寄存器，
    // 仅在锁存切换瞬间极短消隐 (~200ns)，消除可见的暗带扫描线。
    // ====================================================================

    while (true) {
        for (uint8_t row = 0; row < SCAN_ROWS; row++) {
            for (uint8_t bit = 0; bit < COLOR_DEPTH; bit++) {
                uint8_t bitPos = bit + (8 - COLOR_DEPTH);

                // ── 阶段1: 移位 ──────────────────────────────────────
                // 将本行本位的像素数据移入移位寄存器 (128列)
                // 此时输出锁存器仍在显示上一行/位的数据，不受影响
                for (uint16_t col = 0; col < DISPLAY_WIDTH; col++) {
                    uint16_t idxTop = (row * DISPLAY_WIDTH + col) * 3;
                    uint16_t idxBot = ((row + SCAN_ROWS) * DISPLAY_WIDTH + col) * 3;

                    uint32_t dataVal = 0;
                    if ((frameBuffer[idxTop]     >> bitPos) & 1) dataVal |= R1_MASK;
                    if ((frameBuffer[idxTop + 1] >> bitPos) & 1) dataVal |= G1_MASK;
                    if ((frameBuffer[idxTop + 2] >> bitPos) & 1) dataVal |= B1_MASK;
                    if ((frameBuffer[idxBot]     >> bitPos) & 1) dataVal |= R2_MASK;
                    if ((frameBuffer[idxBot + 1] >> bitPos) & 1) dataVal |= G2_MASK;
                    if ((frameBuffer[idxBot + 2] >> bitPos) & 1) dataVal |= B2_MASK;

                    REG_WRITE(GPIO_OUT_W1TC_REG, DATA_MASK);
                    REG_WRITE(GPIO_OUT_W1TS_REG, dataVal);
                    __asm__ __volatile__("nop");
                    REG_WRITE(GPIO_OUT_W1TS_REG, CLK_MASK);
                    __asm__ __volatile__("nop");
                    REG_WRITE(GPIO_OUT_W1TC_REG, CLK_MASK);
                }

                // ── 阶段2: 锁存切换 (不消隐，消除闪烁) ─────────────
                // 直接切换行地址并锁存，OE保持LOW不中断显示
                // 代价: 行切换瞬间可能有极轻微鬼影，但消除了可见闪烁
                setRowAddress(row);
                REG_WRITE(GPIO_OUT_W1TS_REG, LAT_MASK);   // LAT↑ 锁存
                __asm__ __volatile__("nop");
                REG_WRITE(GPIO_OUT_W1TC_REG, LAT_MASK);   // LAT↓

                // ── 阶段3: BCM位权延时 (显示期间) ────────────────────
                // OE保持LOW，显示继续，直到下一次移位开始
                delayMicroseconds(baseDelay << bit);
                // 注意: 不在此处 OE HIGH! 下一次循环的移位阶段会在
                // OE LOW (显示中) 的状态下进行，实现移位与显示的重叠。
            }
        }
        // 帧结束: 让出CPU时间给WiFi栈和idle task
        // 使用 vTaskDelay(1) 确保 idle task 可以运行 (喂系统看门狗)
        vTaskDelay(1);
    }
}

// ==================== 测试图案 ====================
void fillColor(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < DISPLAY_WIDTH * DISPLAY_HEIGHT; i++) {
        frameBuffer[i * 3]     = r;
        frameBuffer[i * 3 + 1] = g;
        frameBuffer[i * 3 + 2] = b;
    }
}

void fillTestPattern() {
    Serial.println("【显示】开机测试: 红→绿→蓝→白，每色0.5秒");

    const uint8_t colors[][3] = {
        {255, 0, 0},     // 红
        {0, 255, 0},     // 绿
        {0, 0, 255},     // 蓝
        {255, 255, 255}  // 白
    };
    const char* names[] = {"红", "绿", "蓝", "白"};

    for (int c = 0; c < 4; c++) {
        fillColor(colors[c][0], colors[c][1], colors[c][2]);
        Serial.printf("【显示】测试: %s\n", names[c]);
        delay(500);
    }

    // 测试结束后清屏
    fillColor(0, 0, 0);
    Serial.println("【显示】测试完成，屏幕已清空");
}

// ==================== CORS 头部 ====================
void addCORSHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// ==================== Web 处理函数 ====================
void handleRoot() {
    addCORSHeaders();
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'></head><body>";
    html += "<h1>ESP32 LED矩阵屏控制器</h1>";
    html += "<p>状态: 运行中</p>";
    html += "<p>分辨率: " + String(DISPLAY_WIDTH) + "x" + String(DISPLAY_HEIGHT) + "</p>";
    html += "<p>色深: " + String(COLOR_DEPTH) + "位 BCM</p>";
    html += "<p>请使用Web前端上传图片</p>";
    html += "</body></html>";
    server.send(200, "text/html; charset=utf-8", html);
}

void handleStatus() {
    addCORSHeaders();
    String json = "{";
    json += "\"status\":\"running\",";
    json += "\"width\":" + String(DISPLAY_WIDTH) + ",";
    json += "\"height\":" + String(DISPLAY_HEIGHT) + ",";
    json += "\"colorDepth\":" + String(COLOR_DEPTH) + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI());
    json += "}";
    server.send(200, "application/json", json);
}

void handleUpload() {
    addCORSHeaders();

    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"仅支持POST方法\"}");
        return;
    }

    String body = server.arg("plain");
    if (body.length() == 0) {
        server.send(400, "application/json", "{\"error\":\"未收到数据\"}");
        Serial.println("【错误】收到空的上传请求");
        return;
    }

    Serial.printf("【图片】收到Base64数据，长度: %d 字节\n", body.length());

    const size_t expectedSize = DISPLAY_WIDTH * DISPLAY_HEIGHT * 3; // 12288
    size_t decodedLen = 0;

    // Base64解码直接写入帧缓冲区
    int ret = mbedtls_base64_decode(
        frameBuffer, sizeof(frameBuffer), &decodedLen,
        (const unsigned char*)body.c_str(), body.length()
    );

    if (ret != 0) {
        server.send(400, "application/json", "{\"error\":\"Base64解码失败\"}");
        Serial.printf("【错误】Base64解码失败，错误码: %d\n", ret);
        return;
    }

    if (decodedLen != expectedSize) {
        Serial.printf("【错误】数据大小不匹配: 期望 %d 字节，实际 %d 字节\n", expectedSize, decodedLen);
        server.send(400, "application/json",
            "{\"error\":\"数据大小不正确，期望" + String(expectedSize) +
            "字节，实际" + String(decodedLen) + "字节\"}");
        return;
    }

    // 应用Gamma校正
    for (size_t i = 0; i < expectedSize; i++) {
        frameBuffer[i] = gammaTable[frameBuffer[i]];
    }

    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"图片已更新\"}");
    Serial.println("【图片】新图片已接收并显示 ✓");
}

void handleClearScreen() {
    addCORSHeaders();
    memset(frameBuffer, 0, sizeof(frameBuffer));
    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"屏幕已清空\"}");
    Serial.println("【显示】屏幕已清空");
}

// ==================== WiFi 连接 ====================
void connectWiFi() {
    Serial.println("【WiFi】开始连接...");
    Serial.printf("【WiFi】SSID: %s\n", ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts > 40) {
            Serial.println("\n【WiFi】连接超时，正在重试...");
            WiFi.disconnect();
            delay(1000);
            WiFi.begin(ssid, password);
            attempts = 0;
        }
    }

    Serial.println();
    Serial.println("【WiFi】连接成功！");
    Serial.printf("【WiFi】IP地址: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("【WiFi】信号强度: %d dBm\n", WiFi.RSSI());
}

// ==================== setup ====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  ESP32 HUB75 LED矩阵屏图片显示系统");
    Serial.println("  分辨率: 128x32 | 1/16扫描 | 手动驱动");
    Serial.println("  色深: 6位BCM (262144色)");
    Serial.println("========================================");

    // 初始化Gamma校正表
    initGammaTable();

    // 初始化HUB75引脚
    initHUB75Pins();

    // 启动显示扫描任务 (Core 1, 独占核心，不受WiFi中断干扰)
    // Core 0: WiFi/BT协议栈 + Arduino loop (Web服务器)
    // Core 1: LED扫描任务 (独占，保证刷新稳定)
    xTaskCreatePinnedToCore(
        displayScanTask,   // 任务函数
        "DisplayTask",     // 任务名称
        4096,              // 堆栈大小
        NULL,              // 参数
        configMAX_PRIORITIES - 1, // 最高优先级，确保不被抢占
        NULL,              // 任务句柄
        1                  // 绑定到Core 1 (独占)
    );
    Serial.println("【系统】显示扫描任务已创建 (Core 1独占)");

    // 开机测试图案 (扫描任务已启动，可以看到显示)
    fillTestPattern();

    // 连接WiFi
    connectWiFi();

    // 启动mDNS服务
    if (MDNS.begin("esp32-led")) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("【mDNS】已启动: http://esp32-led.local");
    }

    // 配置Web服务器路由
    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/upload", HTTP_POST, handleUpload);
    server.on("/clear", HTTP_POST, handleClearScreen);

    // 全局OPTIONS处理 (CORS预检)
    server.onNotFound([]() {
        if (server.method() == HTTP_OPTIONS) {
            addCORSHeaders();
            server.send(204);
        } else {
            addCORSHeaders();
            server.send(404, "application/json", "{\"error\":\"未找到\"}");
        }
    });

    server.begin();

    Serial.println("【服务器】Web服务器已启动，端口: 80");
    Serial.printf("【服务器】上传地址: http://%s/upload\n", WiFi.localIP().toString().c_str());
    Serial.printf("【服务器】状态查询: http://%s/status\n", WiFi.localIP().toString().c_str());
    Serial.println("========================================");
    Serial.println("【系统】初始化完成，等待图片上传...");
    Serial.println("========================================");
}

// ==================== loop ====================
void loop() {
    // WiFi断线重连
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("【WiFi】连接断开，正在重连...");
        WiFi.reconnect();
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
            delay(500);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\n【WiFi】重连成功，IP: %s\n", WiFi.localIP().toString().c_str());
        } else {
            Serial.println("\n【WiFi】重连失败，稍后重试...");
        }
    }

    // 处理Web请求
    server.handleClient();
    delay(2);
}
# 
# CSDN: v_for_van
# WeChat Official Account: 星火在摸鱼 (Xinghuo Zai Moyu)
# GitHub: 1StarStarFire1
# 
# Current Version V1.0 - Implements basic transmit/receive functionality with known limitations
#
# Software UART Implementation
- Supports ESP-IDF platform (tested on ESP32-S3 with ESP-IDF v5.21)
- Configurable baud rate, GPIO pins, and buffer size
- Efficient reception using ring buffer mechanism
- Interrupt-driven reception with polling-based transmission

# 
# CSDN：v_for_van
# 微信公众号：星火在摸鱼
# github：1StarStarFire1
# 
# 当前版本V1.0 - 实现了最基本的收发功能，还有很多欠缺
#
# 软件UART实现
- 支持ESPIDF平台(目前只实验过esp32s3 v5.21版本)
- 支持可配置的波特率、引脚和缓冲区大小
- 使用环形缓冲区实现高效接收
- 支持中断驱动接收和轮询发送

## 使用示例
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "softserial.h"

static const char *TAG = "SoftwareUART";
void app_main() 
{
    SoftwareUART uart = 
    {
        .baud_rate = 38400,
        .rx_buff_size = 512,
        .tx_pin = 11,
        .rx_pin = 12,
    };
    
    software_uart_init(&uart);

    ESP_LOGI(TAG, "Software UART Test Starting...");
    software_uart_tx_str(&uart, "Hello, Software UART!\r\n");

    while (1) 
    {
        uint8_t buffer;
        int len = software_uart_rx_read(&uart, &buffer, 1);
        if (len > 0) 
        {
            ESP_LOGI(TAG, "Received: %02X", buffer);
            char hex_str[5];
            sprintf(hex_str, "%02X", buffer);
            software_uart_tx_str(&uart, hex_str);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
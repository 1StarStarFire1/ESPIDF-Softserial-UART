# 
# CSDN: v_for_van
# WeChat Official Account: �ǻ������� (Xinghuo Zai Moyu)
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
# CSDN��v_for_van
# ΢�Ź��ںţ��ǻ�������
# github��1StarStarFire1
# 
# ��ǰ�汾V1.0 - ʵ������������շ����ܣ����кܶ�Ƿȱ
#
# ���UARTʵ��
- ֧��ESPIDFƽ̨(Ŀǰֻʵ���esp32s3 v5.21�汾)
- ֧�ֿ����õĲ����ʡ����źͻ�������С
- ʹ�û��λ�����ʵ�ָ�Ч����
- ֧���ж��������պ���ѯ����

## ʹ��ʾ��
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
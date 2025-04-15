/* 
 * main.c - 系统启动文件，负责初始化硬件和启动主循环
 * 
 * CSDN：v_for_van
 * 微信公众号：星火在摸鱼
 * github：1StarStarFire1
 * 
 * 一个创建软件串口的example
 * 
 */

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
    // 初始化软件UART
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
        //如果接收到数据
        int len = software_uart_rx_read(&uart, &buffer, 1);
        if (len > 0) 
        {
            //把数据打印出来
            ESP_LOGI(TAG, "Received: %02X", buffer);
            //把数据发送到软件串口
            char hex_str[5];
            sprintf(hex_str, "%02X", buffer);
            software_uart_tx_str(&uart, hex_str);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
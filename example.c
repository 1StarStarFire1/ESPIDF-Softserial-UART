/* 
 * main.c - ϵͳ�����ļ��������ʼ��Ӳ����������ѭ��
 * 
 * CSDN��v_for_van
 * ΢�Ź��ںţ��ǻ�������
 * github��1StarStarFire1
 * 
 * һ������������ڵ�example
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
    // ��ʼ�����UART
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
        //������յ�����
        int len = software_uart_rx_read(&uart, &buffer, 1);
        if (len > 0) 
        {
            //�����ݴ�ӡ����
            ESP_LOGI(TAG, "Received: %02X", buffer);
            //�����ݷ��͵��������
            char hex_str[5];
            sprintf(hex_str, "%02X", buffer);
            software_uart_tx_str(&uart, hex_str);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
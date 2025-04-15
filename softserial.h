/* 
 * CSDN��v_for_van
 * ΢�Ź��ںţ��ǻ�������
 * github��1StarStarFire1
 */

#ifndef _SOFTSERIAL_H_
#define _SOFTSERIAL_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

typedef struct 
{
    uint8_t *buffer;//������ָ��
    uint16_t head;//ͷ
    uint16_t tail;//β
    uint16_t rx_buff_size;//��С
} UART_Buffer;

typedef struct 
{
    uint8_t tx_pin;//��������
    uint8_t rx_pin;//��������
    uint16_t rx_buff_size;//���ջ�������С
    uint32_t baud_rate;//������-Ĭ�������߼���ƽ-������λ
    uint64_t bit_cycles;// ÿλʱ�䣨CPU���ڣ�
    UART_Buffer rx_buffer;//���ɸı�
    portMUX_TYPE lock;//���ɸı�
} SoftwareUART;

void software_uart_init(SoftwareUART *uart);
void software_uart_tx_str(SoftwareUART *uart,const char* str);
int software_uart_rx_read(SoftwareUART *uart, uint8_t* data, size_t size);

#endif // !1




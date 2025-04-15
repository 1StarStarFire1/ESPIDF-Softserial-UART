/* 
 * CSDN: v_for_van
 * WeChat Official Account: Xinghuo Zai Moyu (ÐÇ»ðÔÚÃþÓã)
 * GitHub: 1StarStarFire1
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
    uint8_t *buffer; // Buffer pointer
    uint16_t head;   // Head pointer
    uint16_t tail;   // Tail pointer
    uint16_t rx_buff_size; // Buffer size
} UART_Buffer;

typedef struct 
{
    uint8_t tx_pin; // TX pin number
    uint8_t rx_pin; // RX pin number
    uint16_t rx_buff_size; // RX buffer size (default: 128 bytes)
    uint32_t baud_rate; // Baud rate (default: 38400, 8 data bits, normal logic level)
    uint64_t bit_cycles; // CPU cycles per bit (calculated internally)
    UART_Buffer rx_buffer; // Internal buffer structure (do not modify)
    portMUX_TYPE lock; // Critical section lock (do not modify)
} SoftwareUART;

/* 
 * Initialize software UART instance
 * @param uart Pointer to SoftwareUART structure
 * @note Configures default values if parameters are zero
 */
void software_uart_init(SoftwareUART *uart);

/* 
 * Transmit null-terminated string
 * @param uart UART instance pointer
 * @param str String to send (ASCII only)
 */
void software_uart_tx_str(SoftwareUART *uart,const char* str);

/* 
 * Read data from RX buffer
 * @param uart UART instance pointer
 * @param data Output buffer
 * @param size Maximum bytes to read
 * @return Actual bytes read
 */
int software_uart_rx_read(SoftwareUART *uart, uint8_t* data, size_t size);

#endif // _SOFTSERIAL_H_
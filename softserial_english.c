/* 
 * CSDN: v_for_van
 * WeChat Official Account: Xinghuo Zai Moyu (ÐÇ»ðÔÚÃþÓã)
 * GitHub: 1StarStarFire1
 */

#include "softserial.h"

/* 
 * Calculate bit time in CPU cycles
 * @param baud_rate Target baud rate
 * @return CPU cycles per bit
 * @note Multiply by 2 to compensate for ESP32's GPIO sampling delay and clock division
 */
uint64_t calculate_bit_time(uint32_t baud_rate) 
{
    uint64_t bit_time_us = (1000000UL / baud_rate);
    return 2 * bit_time_us * (CPU_CLK_FREQ / 1000000);  // Convert to CPU cycles  
}

/* 
 * Precise time delay using CPU cycles
 * @param star Pointer to start time (auto-updated)
 * @param bit_cycles Cycles to wait
 */
void wait_time(uint64_t* star, uint64_t bit_cycles)
{
    while(esp_cpu_get_cycle_count() - *star < bit_cycles);
    *star += bit_cycles;
}

/* 
 * RX interrupt service routine (ISR)
 * @param arg Pointer to SoftwareUART instance
 * @note Must be executed within critical section
 */
void IRAM_ATTR software_uart_rx_isr(void* arg) 
{
    SoftwareUART* uart = (SoftwareUART*)arg;
    portBASE_TYPE higher_priority_task_woken = pdFALSE;

    // Enter critical section
    portENTER_CRITICAL_ISR(&uart->lock);
    // Detect start bit (falling edge triggered)
    if (gpio_get_level(uart->rx_pin) == 0) 
    {
        uint8_t data = 0;
        uint64_t star = esp_cpu_get_cycle_count();
        wait_time(&star, (uart->bit_cycles)/2); // Align to middle of first data bit
        if(gpio_get_level(uart->rx_pin) == 0) // Confirm start bit
        {
            // Read 8 data bits (LSB first)
            for (int i = 0; i < 8; i++) 
            {
                wait_time(&star, uart->bit_cycles);  // Sample at center of bit period
                data |= (gpio_get_level(uart->rx_pin) << i);
            }
            // Check stop bit (must be high)
            wait_time(&star, uart->bit_cycles);
            if (gpio_get_level(uart->rx_pin) == 1) // Stop bit validation
            {
                // Store data in ring buffer
                uint16_t next_head = (uart->rx_buffer.head + 1) % uart->rx_buff_size;
                if (next_head != uart->rx_buffer.tail) 
                {
                    uart->rx_buffer.buffer[uart->rx_buffer.head] = data;
                    uart->rx_buffer.head = next_head;
                } 
            }
        }
        
    }
    portEXIT_CRITICAL_ISR(&uart->lock);
}

/* 
 * Initialize software UART
 * @param uart Pointer to SoftwareUART structure
 * @note Default configuration: TX=GPIO11, RX=GPIO12, Baud=38400, Buffer size=128 bytes
 */
void software_uart_init(SoftwareUART *uart) 
{
    if (uart->tx_pin == 0) uart->tx_pin = 11;    // Default TX pin
    if (uart->rx_pin == 0) uart->rx_pin = 12;    // Default RX pin
    if (uart->rx_buff_size == 0) uart->rx_buff_size = 128; // Default buffer size
    if (uart->baud_rate == 0) uart->baud_rate = 38400; // Default baud rate

    // Calculate bit timing
    uart->bit_cycles = calculate_bit_time(uart->baud_rate);

    // Initialize ring buffer
    uart->rx_buffer.buffer = (uint8_t*)malloc(uart->rx_buff_size * sizeof(uint8_t));

    // Configure TX pin as output
    gpio_config_t tx_conf = 
    {
        .pin_bit_mask = (1ULL << uart->tx_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&tx_conf));

    // Configure RX pin as input with interrupt
    gpio_config_t rx_conf = 
    {
        .pin_bit_mask = (1ULL << uart->rx_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Trigger on start bit falling edge
    };
    ESP_ERROR_CHECK(gpio_config(&rx_conf));

    // Initialize buffer pointers
    uart->rx_buffer.head = 0;
    uart->rx_buffer.tail = 0;

    // Initialize critical section lock
    portMUX_INITIALIZE(&uart->lock);

    // Setup interrupt service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(uart->rx_pin, software_uart_rx_isr, (void*)uart));
}

/* 
 * Transmit single byte (8 data bits + start/stop bits)
 * @param uart UART instance pointer
 * @param data 8-bit data to send
 */
void software_uart_tx_byte(SoftwareUART *uart, uint8_t data) 
{
    portENTER_CRITICAL(&uart->lock);

    // Send start bit (low level)
    gpio_set_level(uart->tx_pin, 0);
    uint64_t star = esp_cpu_get_cycle_count(); 
    wait_time(&star, uart->bit_cycles); // Maintain start bit duration

    // Send 8 data bits (LSB first)
    for (int i = 0; i < 8; i++) 
    {
        gpio_set_level(uart->tx_pin, (data >> i) & 0x01);
        wait_time(&star, uart->bit_cycles); // Maintain each data bit duration
    }

    // Send stop bit (high level)
    gpio_set_level(uart->tx_pin, 1);
    wait_time(&star, uart->bit_cycles); // Maintain stop bit duration
    portEXIT_CRITICAL(&uart->lock);
}

/* 
 * Transmit null-terminated string
 * @param uart UART instance pointer
 * @param str String to send
 */
void software_uart_tx_str(SoftwareUART *uart,const char* str) 
{
    while (*str) 
    {
        software_uart_tx_byte(uart,*str++);
    }
}

/* 
 * Read data from RX buffer
 * @param uart UART instance pointer
 * @param data Output buffer
 * @param size Maximum bytes to read
 * @return Actual bytes read
 */
int software_uart_rx_read(SoftwareUART *uart, uint8_t* data, size_t size) 
{
    portENTER_CRITICAL(&uart->lock);

    // Calculate available data
    size_t available = (uart->rx_buffer.head - uart->rx_buffer.tail) % uart->rx_buff_size;
    size_t read_size = (available < size) ? available : size;

    // Read data
    for (size_t i = 0; i < read_size; i++) 
    {
        data[i] = uart->rx_buffer.buffer[uart->rx_buffer.tail];
        uart->rx_buffer.tail = (uart->rx_buffer.tail + 1) % uart->rx_buff_size;
    }

    portEXIT_CRITICAL(&uart->lock);
    return read_size;
}
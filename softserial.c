/* 
 * CSDN��v_for_van
 * ΢�Ź��ںţ��ǻ�������
 * github��1StarStarFire1
 */

#include "softserial.h"

/* 
 * ����ÿλʱ�䣨CPU���ڣ�
 * @param baud_rate ������
 * @return ÿλ��ӦCPU������
 * @note ����2��Ϊ�˲���ESP32��GPIO�����ӳٺͷ�Ƶ����
 */
uint64_t calculate_bit_time(uint32_t baud_rate) 
{
    uint64_t bit_time_us = (1000000UL / baud_rate);
    return 2 * bit_time_us * (CPU_CLK_FREQ / 1000000);  // ת��ΪCPU������  
    //ע�⣺ǰ����Ҫ*һ��2����Ȼ�Ļ������Ӧ�����ڲ��з�Ƶ֮��ģ���̫��
}


/* 
 * ��ȷ�ȴ�ָ��CPU����
 * @param star ָ����ʼʱ���ָ�루�Զ����£�
 * @param bit_cycles ��Ҫ�ȴ���������
 * @notestar ��Ϊ+=bit_cycles
 */
void wait_time(uint64_t* star , uint64_t bit_cycles)
{
    while(esp_cpu_get_cycle_count() - *star < bit_cycles);
    *star += bit_cycles;
}

/* 
 * �����жϷ������̣�ISR��
 * @param arg ���UART�ṹ��ָ��
 * @note �������ٽ����д����ж�
 */
void IRAM_ATTR software_uart_rx_isr(void* arg) 
{
    SoftwareUART* uart = (SoftwareUART*)arg;
    portBASE_TYPE higher_priority_task_woken = pdFALSE;

    // �����ٽ���
    portENTER_CRITICAL_ISR(&uart->lock);
    // �����ʼλ���½��ش�����
    if (gpio_get_level(uart->rx_pin) == 0) 
    {
        uint8_t data = 0;
        uint64_t star = esp_cpu_get_cycle_count();
        wait_time(&star , (uart->bit_cycles)/2);//���Զ���֤��bit��λ��ȡ����
        if(gpio_get_level(uart->rx_pin) == 0)//��ʼλ
        {
            // ��ȡ8λ���ݣ�LSB�� MSB��
            for (int i = 0; i < 8; i++) 
            {
                wait_time(&star , uart->bit_cycles);  // �ȴ���������
                data |= (gpio_get_level(uart->rx_pin) << i);
                // �ȴ���һ����
            }
            // ���ֹͣλ��ӦΪ�ߵ�ƽ��
            wait_time(&star , uart->bit_cycles);
            if (gpio_get_level(uart->rx_pin) == 1) //ֹͣλ
            {
                // ���뻺����
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
 * ��ʼ�����UART
 * @param uart ���UART�ṹ��ָ��
 * @note Ĭ�����ã�TX=11, RX=12, ������38400, ������128�ֽ�
 */
void software_uart_init(SoftwareUART *uart) 
{
    if (uart->tx_pin == 0) uart->tx_pin = 11;    // Ĭ��TX����
    if (uart->rx_pin == 0) uart->rx_pin = 12;    // Ĭ��RX����
    if (uart->rx_buff_size == 0) uart->rx_buff_size = 128; // Ĭ�ϻ�������С
    if (uart->baud_rate == 0) uart->baud_rate = 38400; // Ĭ�ϲ�����

    // ����ÿλʱ��(cpu����)
    uart->bit_cycles = calculate_bit_time(uart->baud_rate);

    //����������
    uart->rx_buffer.buffer = (uint8_t*)malloc(uart->rx_buff_size * sizeof(uint8_t));

    // ���÷�������Ϊ���
    gpio_config_t tx_conf = 
    {
        .pin_bit_mask = (1ULL << uart->tx_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&tx_conf));

    // ���ý�������Ϊ���룬�������ж�
    gpio_config_t rx_conf = 
    {
        .pin_bit_mask = (1ULL << uart->rx_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // �����ʼλ�½���
    };
    ESP_ERROR_CHECK(gpio_config(&rx_conf));

    // ��ʼ�����ջ�����
    uart->rx_buffer.head = 0;
    uart->rx_buffer.tail = 0;

    // ��ʼ���ٽ�����
    portMUX_INITIALIZE(&uart->lock);

    // ��װGPIO�жϷ���
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(uart->rx_pin, software_uart_rx_isr,(void*)uart));//����rx ��ṹ���ֵ
}


/* 
 * ����һ���ֽڣ�8λ���� + ��ʼλ + ֹͣλ��
 * @param uart ���UART�ṹ��ָ��
 * @param data ��Ҫ���͵�8λ����
 */
void software_uart_tx_byte(SoftwareUART *uart, uint8_t data) 
{
    portENTER_CRITICAL(&uart->lock);

    // ������ʼλ���͵�ƽ��
    gpio_set_level(uart->tx_pin, 0);
    //��ʼʱ��
    uint64_t star = esp_cpu_get_cycle_count(); 
    //����һ��bit
    wait_time(&star , uart->bit_cycles);
    // ����8λ���ݣ�LSB�ȷ��ͣ�
    for (int i = 0; i < 8; i++) 
    {
        gpio_set_level(uart->tx_pin, (data >> i) & 0x01);
        //����һ��bit
        wait_time(&star , uart->bit_cycles);
    }

    // ����ֹͣλ���ߵ�ƽ��
    gpio_set_level(uart->tx_pin, 1);
    //����һ��bit
    wait_time(&star , uart->bit_cycles);
    portEXIT_CRITICAL(&uart->lock);
}

/* 
 * �����ַ��������ֽڷ��ͣ�
 * @param uart ���UART�ṹ��ָ��
 * @param str ��Ҫ���͵���NULL��β���ַ���
 */
void software_uart_tx_str(SoftwareUART *uart,const char* str) 
{
    while (*str) 
    {
        software_uart_tx_byte(uart,*str++);
    }
}

/* 
 * �ӽ��ջ�������ȡ����
 * @param uart ���UART�ṹ��ָ��
 * @param data �洢��ȡ���ݵĻ�����
 * @param size ��Ҫ��ȡ������ֽ���
 * @return ʵ�ʶ�ȡ���ֽ���
 */
int software_uart_rx_read(SoftwareUART *uart, uint8_t* data, size_t size) 
{
    //�ٽ���
    portENTER_CRITICAL(&uart->lock);

    //��ȡ�ɶ���������С
    size_t available = (uart->rx_buffer.head - uart->rx_buffer.tail) % uart->rx_buff_size;
    size_t read_size = (available < size) ? available : size;

    //��ȡ��data
    for (size_t i = 0; i < read_size; i++) 
    {
        data[i] = uart->rx_buffer.buffer[uart->rx_buffer.tail];
        uart->rx_buffer.tail = (uart->rx_buffer.tail + 1) % uart->rx_buff_size;
    }

    portEXIT_CRITICAL(&uart->lock);
    return read_size;
}
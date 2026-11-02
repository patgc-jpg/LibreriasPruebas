#include "SimpleUART.h"
#include <esp_log.h>

SimpleUART::SimpleUART(int baud_rate, uart_port_t port, int tx_io, int rx_io, int buff_size)
{
    _port = port;
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(port, buff_size, buff_size, 20, &_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_io, rx_io, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

SimpleUART::~SimpleUART()
{
    uart_driver_delete(_port);
}

int SimpleUART::read(char message_in[], uint32_t message_length)
{
    if (message_length == 0) return -1;

    // Reserve last byte for null terminator
    int len = uart_read_bytes(_port, (uint8_t*)message_in, message_length - 1, 10 / portTICK_PERIOD_MS);
    if (len < 0) {
        ESP_LOGW("SimpleUART", "uart_read_bytes failed");
        return -1;
    }
    message_in[len] = '\0';
    return len;
}

void SimpleUART::write(const char* message_out, uint32_t message_length)
{
    int written = uart_write_bytes(_port, message_out, message_length);
    if (written < 0)
        ESP_LOGW("SimpleUART", "uart_write_bytes failed");
}

int SimpleUART::available()
{
    size_t buffered_size = 0;
    uart_get_buffered_data_len(_port, &buffered_size);
    return (int)buffered_size;
}
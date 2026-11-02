#include "SimpleI2CSlave.h"
#include <esp_log.h>

SimpleI2CSlave::SimpleI2CSlave(uint8_t slave_addr, i2c_port_t port, int sda_io, int scl_io,
                                int buff_size)
{
    _port = port;

    // Zero every field first, then set only what we need.
    i2c_config_t i2c_config = {};
    i2c_config.mode = I2C_MODE_SLAVE;
    i2c_config.sda_io_num = sda_io;
    i2c_config.scl_io_num = scl_io;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.slave.addr_10bit_en = 0;   // 7-bit addressing, the common case
    i2c_config.slave.slave_addr = slave_addr;
    i2c_config.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(port, &i2c_config));
    // Master mode uses 0/0 for rx/tx buffers; slave mode needs real buffers --
    // this is the detail that trips people up switching SimpleI2C -> SimpleI2CSlave.
    ESP_ERROR_CHECK(i2c_driver_install(port, i2c_config.mode, buff_size, buff_size, 0));
}

SimpleI2CSlave::~SimpleI2CSlave()
{
    i2c_driver_delete(_port);
}

int SimpleI2CSlave::read(uint8_t* data, size_t length, uint32_t timeout_ms)
{
    int len = i2c_slave_read_buffer(_port, data, length, timeout_ms / portTICK_PERIOD_MS);
    if (len < 0) {
        ESP_LOGW("SimpleI2CSlave", "read failed");
        return -1;
    }
    return len;
}

int SimpleI2CSlave::write(const uint8_t* data, size_t length, uint32_t timeout_ms)
{
    int written = i2c_slave_write_buffer(_port, data, (int)length, timeout_ms / portTICK_PERIOD_MS);
    if (written < 0)
        ESP_LOGW("SimpleI2CSlave", "write failed");
    return written;
}

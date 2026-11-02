#include "SimpleI2C.h"
#include <esp_log.h>

#define I2C_TIMEOUT_MS 100

SimpleI2C::SimpleI2C(int clock_speed, i2c_port_t port, int sda_io, int scl_io)
{
    _port = port;

    // Zero every field first (including ones newer ESP-IDF versions add,
    // like clk_flags), then set only what we need.
    i2c_config_t i2c_config = {};
    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = sda_io;
    i2c_config.scl_io_num = scl_io;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.master.clk_speed = (uint32_t)clock_speed;
    i2c_config.clk_flags = 0;    // 0 = default clock source

    ESP_ERROR_CHECK(i2c_param_config(port, &i2c_config));
    // rx/tx buffer length is only used in slave mode; 0 for master
    ESP_ERROR_CHECK(i2c_driver_install(port, i2c_config.mode, 0, 0, 0));
}

SimpleI2C::~SimpleI2C()
{
    i2c_driver_delete(_port);
}

void SimpleI2C::write(uint8_t device_addr, const uint8_t* data, size_t length)
{
    esp_err_t err = i2c_master_write_to_device(_port, device_addr, data, length,
                                                I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK)
        ESP_LOGW("SimpleI2C", "write failed (addr 0x%02X): %s", device_addr, esp_err_to_name(err));
}

int SimpleI2C::read(uint8_t device_addr, uint8_t* data, size_t length)
{
    esp_err_t err = i2c_master_read_from_device(_port, device_addr, data, length,
                                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGW("SimpleI2C", "read failed (addr 0x%02X): %s", device_addr, esp_err_to_name(err));
        return -1;
    }
    return (int)length;
}

int SimpleI2C::readRegister(uint8_t device_addr, uint8_t reg, uint8_t* data, size_t length)
{
    // Write the register pointer, then a repeated-start read — the standard
    // pattern for reading sensor/EEPROM registers over I2C.
    esp_err_t err = i2c_master_write_read_device(_port, device_addr, &reg, 1, data, length,
                                                  I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGW("SimpleI2C", "readRegister failed (addr 0x%02X, reg 0x%02X): %s",
                 device_addr, reg, esp_err_to_name(err));
        return -1;
    }
    return (int)length;
}

int SimpleI2C::scan()
{
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(_port, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            ESP_LOGI("SimpleI2C", "Found device at 0x%02X", addr);
            found++;
        }
    }
    return found;
}

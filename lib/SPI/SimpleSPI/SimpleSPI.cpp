#include "SimpleSPI.h"
#include <esp_log.h>
#include <cstring>

SimpleSPI::SimpleSPI(int clock_speed, spi_host_device_t host, int mosi_io, int miso_io,
                      int sclk_io, int cs_io, int mode)
{
    _host = host;

    // Zero every field first, then set only what we need.
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = mosi_io;
    bus_config.miso_io_num = miso_io;
    bus_config.sclk_io_num = sclk_io;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 4096;
    // SPI_DMA_CH_AUTO lets the driver pick a DMA channel automatically --
    // needed for any transfer larger than a few bytes.
    ESP_ERROR_CHECK(spi_bus_initialize(host, &bus_config, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_config = {};
    dev_config.mode = (uint8_t)mode;          // CPOL/CPHA combination, 0-3
    dev_config.clock_speed_hz = clock_speed;
    dev_config.spics_io_num = cs_io;          // driver toggles CS automatically per transfer
    dev_config.queue_size = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(host, &dev_config, &_handle));
}

SimpleSPI::~SimpleSPI()
{
    spi_bus_remove_device(_handle);
    spi_bus_free(_host);
}

void SimpleSPI::transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length)
{
    if (length == 0) return;

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.length = length * 8; // length is in bits, not bytes
    transaction.tx_buffer = tx_data;
    transaction.rx_buffer = rx_data;

    esp_err_t err = spi_device_transmit(_handle, &transaction);
    if (err != ESP_OK)
        ESP_LOGW("SimpleSPI", "transfer failed: %s", esp_err_to_name(err));
}

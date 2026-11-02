#include "SimpleSPISlave.h"
#include <esp_log.h>
#include <cstring>

SimpleSPISlave::SimpleSPISlave(spi_host_device_t host, int mosi_io, int miso_io, int sclk_io,
                                int cs_io, int mode)
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

    spi_slave_interface_config_t slave_config = {};
    slave_config.spics_io_num = cs_io;
    slave_config.flags = 0;
    slave_config.queue_size = 1;     // one transaction in flight at a time -- transfer() is blocking
    slave_config.mode = (uint8_t)mode;

    // Slave mode needs DMA (unlike master mode, which can run without it for
    // small transfers) -- SPI_DMA_CH_AUTO lets the driver pick the channel.
    ESP_ERROR_CHECK(spi_slave_initialize(host, &bus_config, &slave_config, SPI_DMA_CH_AUTO));
}

SimpleSPISlave::~SimpleSPISlave()
{
    spi_slave_free(_host);
}

bool SimpleSPISlave::transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length,
                               TickType_t timeout_ticks)
{
    if (length == 0) return true;

    spi_slave_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.length = length * 8; // bits, not bytes
    transaction.tx_buffer = tx_data;
    transaction.rx_buffer = rx_data;

    // Blocks until the master actually clocks this many bits through, or
    // timeout_ticks elapses -- there is no "poll and see" here, since the
    // slave has no control over when the master decides to talk.
    esp_err_t err = spi_slave_transmit(_host, &transaction, timeout_ticks);
    if (err != ESP_OK) {
        ESP_LOGW("SimpleSPISlave", "transfer failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

#ifndef _SIMPLESPI_H_
#define _SIMPLESPI_H_

#include <driver/spi_master.h>

class SimpleSPI
{
public:
    // clock_speed: bus clock in Hz (e.g. 1000000 for 1 MHz)
    // mode: SPI mode 0-3, sets CPOL/CPHA
    // cs_io: dedicated chip-select pin for this device
    SimpleSPI(int clock_speed, spi_host_device_t host, int mosi_io, int miso_io, int sclk_io,
              int cs_io, int mode = 0);
    ~SimpleSPI();

    // Full-duplex transfer: clocks out tx_data[] while simultaneously clocking
    // in rx_data[]. Pass nullptr for rx_data if you only care about sending,
    // or for tx_data if you only care about receiving (sends zeros).
    void transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length);

private:
    spi_host_device_t _host;
    spi_device_handle_t _handle;
};

#endif // _SIMPLESPI_H_

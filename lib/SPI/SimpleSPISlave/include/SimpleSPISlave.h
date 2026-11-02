#ifndef _SIMPLESPISLAVE_H_
#define _SIMPLESPISLAVE_H_

#include <driver/spi_slave.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// SPI slave: the master drives SCLK and CS; this board only responds when
// selected. There is no "clock_speed" here -- the slave just samples
// whatever clock the master provides. Each call to transfer() blocks until
// the master completes one full-duplex transaction (or times out).
class SimpleSPISlave
{
public:
    SimpleSPISlave(spi_host_device_t host, int mosi_io, int miso_io, int sclk_io, int cs_io,
                    int mode = 0);
    ~SimpleSPISlave();

    // Blocks waiting for the master to clock `length` bytes. tx_data is what
    // gets shifted out to the master while it does so; rx_data receives what
    // the master sends. Pass nullptr for either if you don't need that side.
    //
    // IMPORTANT: tx_data/rx_data must live in DMA-capable memory and length
    // should be a multiple of 4 bytes -- ordinary stack buffers can fail
    // silently on real hardware. Allocate with heap_caps_malloc(len,
    // MALLOC_CAP_DMA) if you hit garbled transfers.
    bool transfer(const uint8_t* tx_data, uint8_t* rx_data, size_t length,
                   TickType_t timeout_ticks = portMAX_DELAY);

private:
    spi_host_device_t _host;
};

#endif // _SIMPLESPISLAVE_H_

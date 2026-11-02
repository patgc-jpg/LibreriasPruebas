#ifndef _SIMPLEI2CSLAVE_H_
#define _SIMPLEI2CSLAVE_H_

#include <driver/i2c.h>

// I2C slave: this board answers to `slave_addr` when a master (running
// SimpleI2C, or any other I2C master) addresses it. Unlike SimpleI2C, there
// is no per-transaction device address to pass in -- the driver handles
// ACK/NACK and addressing for you; you just push bytes into / pull bytes
// out of the driver's internal buffer.
class SimpleI2CSlave
{
public:
    SimpleI2CSlave(uint8_t slave_addr, i2c_port_t port = I2C_NUM_0, int sda_io = -1,
                    int scl_io = -1, int buff_size = 256);
    ~SimpleI2CSlave();

    // Blocks until the master has written `length` bytes, or timeout_ms elapses.
    // Returns the number of bytes actually read (may be less than requested).
    int read(uint8_t* data, size_t length, uint32_t timeout_ms = 1000);

    // Queues `length` bytes for the master to read on its next read request.
    // Returns the number of bytes actually queued.
    int write(const uint8_t* data, size_t length, uint32_t timeout_ms = 1000);

private:
    i2c_port_t _port;
};

#endif // _SIMPLEI2CSLAVE_H_

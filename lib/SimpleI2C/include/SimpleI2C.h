#ifndef _SIMPLEI2C_H_
#define _SIMPLEI2C_H_

#include <driver/i2c.h>

class SimpleI2C
{
public:
    // clock_speed: typically 100000 (standard) or 400000 (fast mode)
    SimpleI2C(int clock_speed, i2c_port_t port = I2C_NUM_0, int sda_io = -1, int scl_io = -1);
    ~SimpleI2C();

    // Write `length` bytes from data[] to the device at 7-bit address `device_addr`
    void write(uint8_t device_addr, const uint8_t* data, size_t length);

    // Read `length` bytes from the device at `device_addr` into data[]
    int read(uint8_t device_addr, uint8_t* data, size_t length);

    // Common sensor pattern: write a register pointer, then read length bytes back
    int readRegister(uint8_t device_addr, uint8_t reg, uint8_t* data, size_t length);

    // Probe every 7-bit address (1-126) and print which ones ACK. Returns count found.
    int scan();

private:
    i2c_port_t _port;
};

#endif // _SIMPLEI2C_H_

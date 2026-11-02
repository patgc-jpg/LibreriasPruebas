#ifndef __viscosimeter_H__
#define __viscosimeter_H__
#include <stdint.h>
#include <driver/gpio.h>
#include "Control.h"
#include "Encoder.h"
#include "SimpleGPIO.h"
#include "SimplePWM.h"
class viscosimeter{
    public:
    viscosimeter();
    ~viscosimeter();
    bool setup(uint8_t buffersize, gpio_num_t pinA, gpio_num_t pinB, gpio_num_t Enable, gpio_num_t LPWM,
                gpio_num_t RPWM, controller TypeControl, discretizationtype TypeDisc,
                float Kp, float Ki, float Kd, float N, float ref,
               float gearRatio = 18.8f, float Ts = 0.02f,
               float u_min = 0.0f, float u_max = 100.0f,
               int PPR = 16, int mode = 4);
    void update();
    void stop();
    float vscoMax();
    float vscoMin();
    float vscoMeas(bool soapok, bool airok);
    void lavarvsco();
    void revolvervsco();
    bool setRef(float NewREF);
    
    private:
    uint8_t _buffersize;
    gpio_num_t _pinA;
    gpio_num_t _pinB;
    gpio_num_t _Enable;
    gpio_num_t _LPWM;
    controller _TypeControl;
    discretizationtype _TypeDisc;
    float _Kp;
    float _Ki;
    float _Kd; 
    float _N;
    float _ref;
    float _gearRatio;
    float _Ts;
    float _u_min;
    float _u_max;
    int _PPR;
    int _mode;
    float _airref;
    static constexpr int _MAX = 50;
    float _buffer[_MAX] = {};
    int _index = 0;
    float _max;
    float _min;
    float _maxvisc = 1600;
    float _minvisc = 0.018;

    Control _PI;
    Encoder _enc;
    SimpleGPIO _en, _lpwm;
    SimplePWM _PWM;
};


#endif //__viscosimeter_H__
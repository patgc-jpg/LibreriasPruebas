#ifndef _Hbridge_
#define _Hbridge_

#include "SimplePWMA.h"
#include <math.h>

class Hbridge
{
public:
    Hbridge();
    void setup(uint8_t in[2], uint8_t ch[2], ledc_timer_config_t *config, float threshold = 0.0f);
    void setDuty(float duty);
    void setStop();
    // threshold es el duty cycle a partir del cual el motor se empieza a mover
private:
    SimplePWMA _in[2];
    float _m, _b;
};

#endif //_Hbridge_H_

//click derecho- format doc
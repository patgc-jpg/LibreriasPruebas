#ifndef __Hbridge__
#define __Hbridge__
#include <stdint.h>
#include <math.h>
#include "SimplePWM.h"

class Hbridge
{
    public:
        Hbridge();
        void setup(uint8_t in[2], uint8_t ch[2], TimerConfig *config, float thereshold = 15.0f);
        void setDuty(float duty);
        private:

        SimplePWM _in[2];
        float _m, _b;
        TimerConfig *TimerconfigHbridge;
};


#endif
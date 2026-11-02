#include "Hbridge.h"

Hbridge::Hbridge()
{
    
}

void Hbridge::setup(uint8_t in[2], uint8_t ch[2], TimerConfig *timer_config, float threshold)
{
    _in[0].setup(in[0], ch[0], timer_config);
    _in[1].setup(in[1], ch[1], timer_config);
    _b = threshold; //tu valor inicial del duty cycle
    _m = (100.0f - _b)/100.0f;
}

//un duty negativos se refiere a la direccion

void Hbridge::setDuty(float duty)
{
    float adj_duty = _m * fabs(duty) +_b; //mx + b    fabs te da valor absoluto del float
    if(duty<0){
        _in[0].setDuty(adj_duty);
        _in[1].setDuty(0);

    }
    else{
        _in[1].setDuty(adj_duty);
        _in[0].setDuty(0);
    }
}
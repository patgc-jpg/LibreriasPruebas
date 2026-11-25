#include "Hbridge.h"

Hbridge::Hbridge()
{
}

void Hbridge::setup(uint8_t in[2], uint8_t ch[2], ledc_timer_config_t *config, float threshold)
{
    (void)config;
    _in[0].setup(in[0], ch[0], 0, LEDC_TIMER_8_BIT, 25000, LEDC_TIMER_2, LEDC_LOW_SPEED_MODE);
    _in[1].setup(in[1], ch[1], 0, LEDC_TIMER_8_BIT, 25000, LEDC_TIMER_2, LEDC_LOW_SPEED_MODE);
    _b = threshold;
    _m = (100.0f - _b) / 100.0f;
}

void Hbridge::setDuty(float duty)
{
    if (duty == 0.0f) {
        _in[0].setDuty(0.0f);
        _in[1].setDuty(0.0f);
        return;
    }

    if (duty > 100.0f) duty = 100.0f;
    if (duty < -100.0f) duty = -100.0f;

    float adj_duty = _m * fabsf(duty) + _b;
    if (duty < 0.0f) {
        _in[0].setDuty(adj_duty);
        _in[1].setDuty(0.0f);
    } else {
        _in[0].setDuty(0.0f);
        _in[1].setDuty(adj_duty);
    }
}

void Hbridge::setStop(){
    _in[0].setDuty(100.0f);
    _in[1].setDuty(100.0f);
}

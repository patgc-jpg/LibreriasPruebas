#include "PID.h"

PID::PID(){}

void PID::setup(float gains[3], float dt){
    _kp = gains[0];
    _ki = gains[1];
    _kd = gains[2];
    _dt = dt;
}
float PID::calculate(float error)
{
    float u = _kp*error;
    _prev_integ += _dt*(error+_prev_error);
    u += _ki*_prev_integ;
    u += _kd*error - _prev_error/_dt;
    _prev_error = error;
    return u;

}
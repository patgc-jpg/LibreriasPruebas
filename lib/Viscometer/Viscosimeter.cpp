#include "Viscosimeter.h"
#include "Control.h"
#include "Encoder.h"
#include "SimplePWM.h"
#include "SimpleGPIO.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

viscosimeter::viscosimeter(){
};

viscosimeter::~viscosimeter(){};

bool viscosimeter::setup(uint8_t buffersize, gpio_num_t pinA, gpio_num_t pinB, gpio_num_t Enable, gpio_num_t LPWM,
                gpio_num_t RPWM, controller TypeControl, discretizationtype TypeDisc,
                         float Kp, float Ki, float Kd, float N, float ref,
                         float gearRatio, float Ts, float u_min, float u_max,
                         int PPR, int mode){
_enc.setup(pinA, pinB,gearRatio,PPR, mode, Ts);
_PI.setup(TypeControl, TypeDisc ,Ts, u_min, u_max);
_PI.setGains(Kp,Ki,Kd, N);
_PI.setRef(ref);
_en.setup(Enable, GPIO_MODE_OUTPUT, GPIO_FLOATING);
_lpwm.setup(LPWM, GPIO_MODE_OUTPUT, GPIO_FLOATING);
_PWM.setup(RPWM, 0, 0, LEDC_TIMER_12_BIT, 5000);
_lpwm.set(0);
_en.set(1);
_buffersize = buffersize;
if (_buffersize < 1) _buffersize = 1;
if (_buffersize > _MAX) _buffersize = _MAX;
_pinA = pinA;
_pinB = pinB;
_Enable = Enable;
_LPWM = LPWM;
_TypeControl = TypeControl;
_TypeDisc = TypeDisc;
_Kp = Kp;
_Ki = Ki;
_Kd = Kd;
_N = N;
_ref = ref;
_gearRatio = gearRatio;
_Ts = Ts;
_u_min = u_min;
_u_max = u_max;
_PPR = PPR;
_mode = mode;
return true;

};

void viscosimeter::update(){
    _enc.update();
    float rpm = _enc.getRPM(RPMtype::FILTERED);
    float u = _PI.calculate(rpm);
    _PWM.setDuty(u);
    _buffer[_index] = _PI.getCorrection(corrections::I);
    _index = (_index + 1) % _buffersize;

};

void viscosimeter::stop(){
    _PWM.setDuty(0);
    _PI.reset();
    _PI.setRef(_ref);
    for(int i = 0; i < _MAX; i++)
        _buffer[i] = 0;
    _index = 0;
};

// Promedio de las ultimas _buffersize correcciones integrales
float viscosimeter::vscoMax(){
    float sum = 0;
    for(int i = 0; i < _buffersize; i++)
        sum += _buffer[i];
    _max = sum/_buffersize;
    return _max;
};
float viscosimeter::vscoMin(){
    float sum = 0;
    for(int i = 0; i < _buffersize; i++)
        sum += _buffer[i];
    _min = sum/_buffersize;
    return _min;
};
float viscosimeter::vscoMeas(bool soapok, bool airok){
    float sum = 0;
    if(soapok && airok && _max > _min){
        for(int i = 0; i < _buffersize; i++)
        sum += _buffer[i];
        float meas = sum/_buffersize;
        printf("Correccion %0.2f \n", meas);
        return (meas - _min) * (_maxvisc - _minvisc) / (_max - _min) + _minvisc;
    }
    else{
        return 0;
    }
        
    }
    
    
void viscosimeter::lavarvsco(){
    _PI.setRef(150.0f);
    _enc.update();
    float rpm = _enc.getRPM(RPMtype::FILTERED);
    float u = _PI.calculate(rpm);
    _PWM.setDuty(u);
}

void viscosimeter::revolvervsco(){
    _PI.setRef(160.0f);
    _enc.update();
    float rpm = _enc.getRPM(RPMtype::FILTERED);
    float u = _PI.calculate(rpm);
    _PWM.setDuty(u);
    //Aquí estaria muy duro poner que se suba y que se baje un poquito el nema para que revuelva mejor.
}
bool viscosimeter::setRef(float NewREF){
    if(NewREF >= 50 && NewREF <=180){
        _ref = NewREF;
        _PI.setRef(NewREF);
        return true;
    }
    return false;
}
#include "Control.h"

Control::Control(){};

bool Control::setup(controller TypeControl, discretizationtype TypeDisc, float Ts, float u_min, float u_max){
    if(u_min >= u_max || Ts <= 0){
        return false;
    };
    _TypeControl = TypeControl;
    _Ts = Ts;
    _u_min = u_min;
    _u_max = u_max;
    reset();
    _TypeDiscretizaton = TypeDisc;
    return true;
};

bool Control::setGains(float Kp, float Ki, float Kd, float N){
    if(Kp < 0 || Ki < 0 || Kd < 0 || N < 0)
        return false;
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _N = N;
    return true;
}

bool Control::setRef(float refRPM){
    if(refRPM < 0){
        return false;
    }
    _refRPM = refRPM;
    return true;
}

float Control::calculate(float Rpm_Y){
    if (_Ts <= 0) return 0.0f;

    float e = _refRPM - Rpm_Y;
    float dy = Rpm_Y - _rpmPast;
    float uIntegNew = 0.0f;

    _uProp = _Kp*e;

    if(_TypeDiscretizaton == discretizationtype::Euler){
        if(_TypeControl == controller::P){
            _uDeriv = 0.0f;
        }
        else if(_TypeControl == controller::PI){
            uIntegNew = _uInteg + _Ki*_Ts*e;
            _uDeriv = 0.0f;
        }
        else if(_TypeControl == controller::PID){
            uIntegNew = _uInteg + _Ki*_Ts*e;
            _uDeriv = -_Kd*dy/_Ts;
        }
        else if(_TypeControl == controller::PIDF){
            uIntegNew = _uInteg + _Ki*_Ts*e;
            float a = 1.0f/(1.0f + _N*_Ts);
            _uDeriv = a*_uDeriv - (1.0f - a)*(_Kd/_Ts)*dy;
        }
    }
    else if(_TypeDiscretizaton == discretizationtype::Tustin){
        if(_TypeControl == controller::P){
            _uDeriv = 0.0f;
        }
        else if(_TypeControl == controller::PI){
            uIntegNew = _uInteg + 0.5f*_Ki*_Ts*(e + _ePast);
            _uDeriv = 0.0f;
        }
        else if(_TypeControl == controller::PID){
            uIntegNew = _uInteg + 0.5f*_Ki*_Ts*(e + _ePast);
            _uDeriv = -_uDeriv - (2.0f*_Kd/_Ts)*dy;
        }
        else if(_TypeControl == controller::PIDF){
            uIntegNew = _uInteg + 0.5f*_Ki*_Ts*(e + _ePast);
            float a = (2.0f - _N*_Ts) / (2.0f + _N*_Ts);
            float b = (2.0f*_Kd*_N)  / (2.0f + _N*_Ts);
            _uDeriv = a*_uDeriv - b*dy;
        }
    }
    else{
        return 0.0f;
    }

    float u = _uProp + uIntegNew + _uDeriv;
    if(u > _u_max)
        u = _u_max;
    else if(u < _u_min)
        u = _u_min;
    else
        _uInteg = uIntegNew;

    _rpmPast = Rpm_Y;
    _ePast = e;
    _uTotal = u;
    return u;
};

float Control::getCorrection(corrections action){
    
    if(action == corrections::P)
        return _uProp;
    else if(action == corrections::I)
        return _uInteg;
    else if(action == corrections::D)
        return _uDeriv;
    else if(action == corrections::TOTAL)
        return _uTotal;
    else
        return 0.0f;
};

void Control::reset(){
    _uProp = _uInteg = _uDeriv = _uTotal = _rpmPast = _ePast = 0.0f;
}

float Control::getError(float Rpm_Y){
    _error = _refRPM - Rpm_Y;
    return _error;
};



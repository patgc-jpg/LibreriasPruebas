#pragma once
#include <stdint.h>


enum class controller {P, PI, PID, PIDF};
enum class corrections {P, I, D, TOTAL};
enum class discretizationtype {Euler, Tustin};

class Control{
    public:
        Control();
        bool setup(controller TypeControl, discretizationtype  TypeDisc, float Ts = 0.02f, float u_min = 0.0f, float u_max = 100.0f);
        bool setGains(float Kp, float Ki, float Kd, float N);
        bool setRef(float refRPM);
        float calculate(float Rpm_Y);
        float getCorrection(corrections action = corrections::TOTAL);
        float getError(float Rpm_Y);
        void reset();
    private:
        float _Ts;
        float _u_min;
        float _u_max;
        float _Kp= 0.0f;
        float _Ki = 0.0f;
        float _Kd = 0.0f;
        float _N = 0.0f;
        float _rpmPast = 0.0f;
        float _uProp = 0.0f;
        float _uInteg = 0.0f;
        float _uDeriv = 0.0f;
        float _uTotal = 0.0f;
        float _ePast = 0.0;
        float _error = 0.0;
        float _uDpast = 0.0;
        controller _TypeControl = controller::PI;
        discretizationtype _TypeDiscretizaton = discretizationtype::Tustin;
        float _refRPM = 0.0f;
};
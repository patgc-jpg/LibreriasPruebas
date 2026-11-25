#ifndef __PID_H__
#define __PID_H__

class PID
{
    public:
        void setup(float gains[3], float dt);
        float calculate(float error);
    private:
        float _kp;
        float _ki;
        float _kd;
        float _dt;
        float _prev_error = 0;
        float _prev_integ = 0;
    
};


#endif // __PID_H__
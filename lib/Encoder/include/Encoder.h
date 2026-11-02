#pragma once
#include <esp_timer.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <driver/pulse_cnt.h>


enum class RPMtype {FILTERED, RAW};
enum class FilterType {MA, EMA};

class Encoder {
    public:
        Encoder();
        ~Encoder();
        bool setup (gpio_num_t pinA, gpio_num_t pinB,  float gearRatio = 50.0f, int PPR = 16, int mode = 4, float Ts = 0.02f);
        void update();
        void reset();
        float getRPM(RPMtype getType = RPMtype::FILTERED);
        void setFilter(float param, FilterType filtertype = FilterType::MA); 
    private:
        gpio_num_t _pinA;
        gpio_num_t _pinB;
        float _gearRatio;
        int _PPR;
        int _mode;
        float _Ts;
        float _rpmRaw;
        float _rpmFiltered;
        int64_t _lastTime;
        int _lastCount;
        pcnt_unit_handle_t _unit;
        float _cpr;
        FilterType _filterType = FilterType::MA;  
        int _n = 2;
        float _alpha = 0.3f;
        static constexpr int _MAX = 16;
        float _buffer[_MAX] = {};
        int _index = 0;
        pcnt_channel_handle_t _chA = NULL;
        pcnt_channel_handle_t _chB = NULL;
        
            
};
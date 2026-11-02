#pragma once
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "SimplePWM.h"

struct Note{
    uint16_t freq;
    uint16_t ms;

};

class Buzzer{
    public:
        Buzzer();
        void setup(int gpio, uint8_t channel, ledc_timer_t timer = LEDC_TIMER_3);
        void volume(uint8_t percentage);
        void play(const Note* song, size_t len);
        void stop();
        bool isPlaying();   
    private:
        static void _taskEntry(void* arg);
        void _taskLoop();

        SimplePWM _pwm;
        float _dutyPct = 50.0f;
        uint16_t _gapMs = 20;
        TaskHandle_t _task = nullptr;

        const Note* volatile _song = nullptr;
        volatile size_t _len = 0;
        volatile bool _playing = false;
        volatile bool _stopReq = false;
};

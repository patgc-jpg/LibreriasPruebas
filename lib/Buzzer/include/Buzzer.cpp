#include "Buzzer.h"
Buzzer::Buzzer(){};

void Buzzer::setup(int gpio, uint8_t channel, ledc_timer_t timer){
    if(_task != nullptr)
        return;
    else{
        _pwm.setup(gpio, channel, 0, 10U, 1000UL, timer);
        xTaskCreatePinnedToCore( //Esta parte hay que tenerla en cuenta cuando creemos la state machine, una peque;a guia de freetros
                _taskEntry,   // 1. función que ejecuta la tarea
                "buzzer",     // 2. nombre
                2048,         // 3. stack en bytes
                this,         // 4. parámetro que recibe la función
                1,            // 5. prioridad
                &_task,       // 6. dónde guardar el handle
                1             // 7. core
);
    };
};

void Buzzer::volume(uint8_t percentage){
    if(percentage > 100)
        percentage = 100;
    _dutyPct = percentage*0.5f;
};

void Buzzer::_taskEntry(void* arg)
{
    Buzzer* self = static_cast<Buzzer*>(arg);
    self->_taskLoop();
}

void Buzzer::play(const Note* song, size_t len){
    _song = song;
    _len = len;
    _stopReq = false;
    _playing = true;
    xTaskNotifyGive(_task);
}



void Buzzer::stop(){
    _stopReq = true;
    xTaskNotifyGive(_task);
}

bool Buzzer::isPlaying(){
    return _playing;
}

void Buzzer::_taskLoop(){
    while (true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(_stopReq){
            _pwm.setDuty(0);
            _playing = false;
            continue;
        }
        _playing = true;
        bool interrupted = false;

        for(size_t i = 0; i < _len; i++){
            uint16_t freq = _song[i].freq;
            uint16_t ms   = _song[i].ms;

            if(freq == 0){
                _pwm.setFrequency(freq);
                _pwm.setDuty(_dutyPct);
            } else {
                _pwm.setFrequency(freq);
                _pwm.setDuty(_dutyPct);

            }
            uint16_t on = (ms > _gapMs) ? ms - _gapMs : ms;

             if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(on)) != 0) {
                interrupted = true;
                break;
             }
            _pwm.setDuty(0);
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms - on)) != 0) {
                interrupted = true;
                break;
            }
        }
        _pwm.setDuty(0);

        if (interrupted && !_stopReq) {
            xTaskNotifyGive(xTaskGetCurrentTaskHandle());
        } else {
            _playing = false;
        }
    }
}

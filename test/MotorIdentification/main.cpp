#include "definition.h"

extern "C" void app_main(){
    uint8_t in[2] = {AIN1, AIN2};
    uint8_t ch[2] = {0,1};
    motor.setup(in, ch, nullptr);

    uint8_t encpins[2] = {EncoderA, EncoderB};
    enc.setup(encpins, DEG_PER_EDGE);


    float pwmarray[14] = {20, 35, 50, 70, 55, 30, 18, -20, -35, -50, -70, -55, -30, -18};
    
    esp_task_wdt_deinit();
    uint64_t timer = 0;

    uint64_t start = esp_timer_get_time();
    uint64_t lastchange = start;
    uint64_t lastprint = start;

    while(true){
        for(int i = 0; i <14; i ++){
            motor.setDuty(pwmarray[i]);
            enc.setup(encpins, DEG_PER_EDGE);
            timer = esp_timer_get_time();
            while(timer-lastchange < changetime){
                if(timer - lastprint >= printtime){
                    lastprint += printtime;
                    printf("%.3f, %.0f, %0.1f \r \n", (timer-start)/1e6 , pwmarray[i], enc.getSpeed() / 6.0f );
                }
                timer = esp_timer_get_time();
            }
            lastchange += changetime;
        }
    }
};
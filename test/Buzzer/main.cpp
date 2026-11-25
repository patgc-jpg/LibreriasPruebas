#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Buzzer.h"
#include "audios/spiderman.h"

Buzzer buz;

extern "C" void app_main()
{
    buz.setup(25, 1);
    buz.volume(80);

    buz.play(spiderman, spiderman_len);

    while (true) {
        printf("isPlaying = %d\n", buz.isPlaying());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
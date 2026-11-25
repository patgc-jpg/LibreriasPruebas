#include <stdio.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <stdint.h>
#include <stdbool.h>
#include "Hbridge.h"
#include "Control.h"
#include "QuadratureEncoder.h"

#define EncoderA GPIO_NUM_19
#define EncoderB GPIO_NUM_18
#define AIN1 GPIO_NUM_4
#define AIN2 GPIO_NUM_17
#define changetime 1500000
#define printtime 5000
#define PPR 16
#define Reduction 18.8f
#define DEG_PER_EDGE (360.0f / (2 * Reduction * PPR))

Hbridge motor;
QuadratureEncoder enc;



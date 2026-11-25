#include <esp_timer.h>
#include <stdio.h>
#include <stdbool.h>
#include <esp_task_wdt.h>
#include <string.h>

#include "QuadratureEncoder.h"
#include "Control.h"
#include "Hbridge.h"
#include "SimpleUART.h"

#define EncoderA GPIO_NUM_19
#define EncoderB GPIO_NUM_18
#define AIN1     GPIO_NUM_4
#define AIN2     GPIO_NUM_17

#define PPR 16
#define Reduction 18.8f
#define DEG_PER_EDGE (360.0f / (2 * Reduction * PPR))

#define MOTOR_THRESHOLD 17.0f

#define CONTROL_TS_US   5000
#define LINK_TIMEOUT_US 1000000


Hbridge motor;
QuadratureEncoder enc;
Control pidSpeed;
Control pidPos;
SimpleUART uart(115200);


int   mode = 0; // 0 PWM, 1 speed, 2 pos
float ref = 0;
int   wave = 0; // 0 none, 1 step, 2 square, 3 sine, 4 triangular
float amp = 0, freq = 0, offset = 0;
int   telem_ms = 20;
float relay_on = 0, relay_off = 0;
bool  anim_request = false;
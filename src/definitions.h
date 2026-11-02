#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <esp_task_wdt.h>
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include "Encoder.h"
#include "SimplePWM.h"
#include "SimpleGPIO.h"
#include "Control.h"
#include "Viscosimeter.h"
#include "SimpleUART.h"

#define EncoderA GPIO_NUM_19
#define EncoderB GPIO_NUM_18
#define sampletime 20000
#define Enable GPIO_NUM_13
#define RPWM GPIO_NUM_27
#define LPWM GPIO_NUM_26
#define Kp 0.2328f
#define Ki 2.6607f
#define T1m 10000000.0f
#define Tlavado 9000000.0f


enum state{
    INACTIVE,
    PUREAIR,
    PURESOAP,
    SOAPMEAS,
    WASH,
    STIRR
};

viscosimeter vsco;
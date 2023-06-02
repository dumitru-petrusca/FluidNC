#pragma once

#include <cstdint>
#include <esp32-hal-gpio.h>  // gpio
#include "Protocol.h"

#define INDEX_PIN 32
#define INDEX_EDGE_TYPE Pin::RISING_EDGE

void     synchro_init();
void     synchro_wait_for_index();
double   synchro_spindle_rpm();

#pragma once

#include <cstdint>
#include "Driver/fluidnc_gpio.h"
#include <driver/pcnt.h>

#define MAX_ESP32_ENCODERS PCNT_UNIT_MAX
#define _INT16_MAX 32766
#define _INT16_MIN -32766

enum EncoderType {
    // Either the rising or falling edge of channel A is counted.
    X1,
    // Both the rising and falling edges of channel A are counted.
    X2,
    // Both the rising and falling edges of channels A and B are counted.
    X4
};

class ESP32Encoder {
public:
    ESP32Encoder(gpio_num_t a_pin, gpio_num_t b_pin, enum EncoderType encoder_type = X1, uint16_t filter_value = 250);
    int32_t getCount();
    int32_t getCountAndClear();
    void    pause();
    void    resume();

    pcnt_unit_t      unit           = (pcnt_unit_t)-1;
    volatile int32_t overflow_count = 0;

private:
    void maybe_attach_isr() const;
    void setFilter(uint16_t value);

    static bool          attachedInterrupt;
    static ESP32Encoder* encoders[MAX_ESP32_ENCODERS];
};

#pragma once
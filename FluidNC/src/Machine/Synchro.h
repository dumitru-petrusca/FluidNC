#pragma once

#include <cstdint>
#include <esp32-hal-gpio.h>  // gpio
#include "Protocol.h"
#include "Configuration/Configurable.h"

namespace Machine {

    class Synchro : public Configuration::Configurable {
    public:
        Synchro() = default;
        void   init();
        bool   exists();
        void   wait_for_index();
        double spindle_rpm();

        pinnum_t          _indexPinNum;
        volatile uint64_t _last_encoder_time;
        volatile double   _rpm = 0;

    private:
        void group(Configuration::HandlerBase& handler) override;

        Pin _indexPin;
    };

}
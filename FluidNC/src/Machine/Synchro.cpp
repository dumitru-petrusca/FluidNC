#include <Arduino.h>
#include "Synchro.h"
#include "Planner.h"
#include "System.h"
#include "Driver/fluidnc_gpio.h"

namespace Machine {

    void Synchro::group(Configuration::HandlerBase& handler) {
        handler.item("index_pin", _indexPin);
        _indexPinNum = _indexPin.getNative(Pin::Capabilities::Input);
    }

    void IRAM_ATTR encoder_isr(void* arg) {
        auto*         synchro = (Synchro*)arg;
        unsigned long time    = micros();
        if (digitalRead(synchro->_indexPinNum) == HIGH) {
            uint64_t dt                 = time - synchro->_last_encoder_time;
            synchro->_rpm               = 60 * 1e6 / dt;
            synchro->_last_encoder_time = time;
        }
    }

    void Synchro::init() {
        if (exists()) {
            log_info("Synchro Index:" << _indexPinNum);
            gpio_reset_pin(static_cast<gpio_num_t>(_indexPinNum));
            gpio_mode(_indexPinNum, true, false, false, false, false);
            attachInterruptArg(digitalPinToInterrupt(_indexPinNum), encoder_isr, this, RISING);
        }
    }

    bool Synchro::exists() {
        return _indexPinNum != 0;
    }

    // Monitors index pin state in a tight loop and waits for the rising edge.
    void Synchro::wait_for_index() {
        sys.set_state(State::Sync);
        int value1 = digitalRead(_indexPinNum);
        while (true) {
            int value2 = digitalRead(_indexPinNum);
            if (value1 == LOW && value2 == HIGH) {  // RISING edge
                sys.set_state(State::Idle);
                return;
            }
            value1 = value2;
            delay_us(100);
        }
    }

    double Synchro::spindle_rpm() {
        return _rpm;
    }
}

#include "Synchro.h"
#include "Planner.h"
#include "System.h"
#include "Driver/fluidnc_gpio.h"

#define INDEX_PIN_STATE (INDEX_EDGE_TYPE == Pin::RISING_EDGE ? HIGH : LOW)

volatile uint64_t last_encoder_time;
volatile double   rpm = 0;

void IRAM_ATTR encoder_isr(void* arg) {
    unsigned long time = micros();
    if (digitalRead(INDEX_PIN) == INDEX_PIN_STATE) {
        uint64_t dt       = time - last_encoder_time;
        rpm               = 60 * 1e6 / dt;
        last_encoder_time = time;
    }
}

void synchro_init() {
    gpio_reset_pin(static_cast<gpio_num_t>(INDEX_PIN));
    gpio_mode(INDEX_PIN, true, false, false, false, false);
    //TODO-dp, THIS WAS CHANGED AND NEEDS TO BE TESTED AGAIN
    attachInterruptArg(digitalPinToInterrupt(INDEX_PIN), encoder_isr, nullptr, RISING);
}

// Monitors index pin state in a tight loop and waits for the rising edge.
void synchro_wait_for_index() {
    sys.state  = State::Sync;
    int value1 = digitalRead(INDEX_PIN);
    while (true) {
        int value2 = digitalRead(INDEX_PIN);
        if (value1 == LOW && value2 == HIGH) {  // RISING edge
            sys.state = State::Idle;
            return;
        }
        value1 = value2;
        delay_us(100);
    }
}

double synchro_spindle_rpm() {
    return rpm;
}

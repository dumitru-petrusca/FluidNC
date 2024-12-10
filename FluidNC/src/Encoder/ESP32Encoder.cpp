#include "ESP32Encoder.h"
#include <soc/soc_caps.h>
#include <soc/pcnt_struct.h>
#include "esp_log.h"
#include "esp_ipc.h"

static const char*  TAG_ENCODER = "ESP32Encoder";
static portMUX_TYPE spinlock    = portMUX_INITIALIZER_UNLOCKED;
#define _ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&spinlock)
#define _EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&spinlock)

bool          ESP32Encoder::attachedInterrupt            = false;
ESP32Encoder* ESP32Encoder::encoders[MAX_ESP32_ENCODERS] = { NULL };

// Decode what PCNT's unit originated an interrupt and pass this information together with the event type the main program using a queue.
static void esp32encoder_pcnt_intr_handler(void* arg) {
    ESP32Encoder* encoder = static_cast<ESP32Encoder*>(arg);
    pcnt_unit_t   unit    = encoder->unit;
    _ENTER_CRITICAL();
    if (PCNT.status_unit[unit].h_lim_lat) {
        encoder->overflow_count += _INT16_MAX;
        pcnt_counter_clear(unit);
    } else if (PCNT.status_unit[unit].l_lim_lat) {
        encoder->overflow_count += _INT16_MIN;
        pcnt_counter_clear(unit);
    }
    _EXIT_CRITICAL();
}

ESP32Encoder::ESP32Encoder(gpio_num_t a_pin, gpio_num_t b_pin, enum EncoderType encoder_type, uint16_t filter_value) {
    int index = 0;
    for (; index < MAX_ESP32_ENCODERS; index++) {
        if (ESP32Encoder::encoders[index] == NULL) {
            encoders[index] = this;
            break;
        }
    }
    if (index == MAX_ESP32_ENCODERS) {
        ESP_LOGE(TAG_ENCODER, "Too many encoders, FAIL!");
        return;
    }

    this->unit = (pcnt_unit_t)index;

    //Set up the IO state of the pins
    gpio_pad_select_gpio(a_pin);
    gpio_pad_select_gpio(b_pin);
    gpio_set_direction(a_pin, GPIO_MODE_INPUT);
    gpio_set_direction(b_pin, GPIO_MODE_INPUT);

    // Encoder PCNT configuration
    pcnt_config_t channel0_config = {
        .pulse_gpio_num = a_pin,                                                 // Rotary Encoder Chan A
        .ctrl_gpio_num  = b_pin,                                                 // Rotary Encoder Chan B
        .lctrl_mode     = PCNT_MODE_KEEP,                                        // Rising A on HIGH B = CW Step
        .hctrl_mode     = PCNT_MODE_REVERSE,                                     // Rising A on LOW B = CCW Step
        .pos_mode       = encoder_type == X1 ? PCNT_COUNT_DIS : PCNT_COUNT_DEC,  // Count Only On Rising-Edges
        .neg_mode       = PCNT_COUNT_INC,                                        // Discard Falling-Edge
        .counter_h_lim  = _INT16_MAX,
        .counter_l_lim  = _INT16_MIN,
        .unit           = unit,
        .channel        = PCNT_CHANNEL_0,
    };
    pcnt_config_t channel1_config = {
        .pulse_gpio_num = b_pin,                                                       // make prior control into signal
        .ctrl_gpio_num  = a_pin,                                                       // and prior signal into control
        .lctrl_mode     = encoder_type == X4 ? PCNT_MODE_REVERSE : PCNT_MODE_DISABLE,  // disabling channel 1
        .hctrl_mode     = encoder_type == X4 ? PCNT_MODE_DISABLE : PCNT_MODE_KEEP,     // disabling channel 1
        .pos_mode       = encoder_type == X4 ? PCNT_COUNT_DEC : PCNT_COUNT_DIS,        // disabling channel 1
        .neg_mode       = encoder_type == X4 ? PCNT_COUNT_INC : PCNT_COUNT_DIS,        // disabling channel 1
        .counter_h_lim  = _INT16_MAX,
        .counter_l_lim  = _INT16_MIN,
        .unit           = unit,
        .channel        = PCNT_CHANNEL_1,
    };

    // Configure the counters
    pcnt_unit_config(&channel0_config);       // Configure PCNT channel 0
    pcnt_unit_config(&channel1_config);       // Configure PCNT channel 1
    setFilter(filter_value);                  // Filter out bounces and noise
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);  // Enable event on maximum limit value
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);  // Enable event on minimum limit value
    pcnt_counter_pause(unit);                 // Initial PCNT init
    maybe_attach_isr();                       // Register ISR service and enable interrupts for PCNT unit

    // Add ISR handler for this unit
    if (pcnt_isr_handler_add(unit, esp32encoder_pcnt_intr_handler, this) != ESP_OK) {
        ESP_LOGE(TAG_ENCODER, "Encoder install interrupt handler for unit %d failed", unit);
    }

    pcnt_counter_clear(unit);   // Clear counter
    pcnt_intr_enable(unit);     // Enable interrupts
    pcnt_counter_resume(unit);  // Start the counter
}

void ESP32Encoder::maybe_attach_isr() const {
    if (!attachedInterrupt) {
        if (pcnt_isr_service_install(0) != ESP_OK) {
            ESP_LOGE(TAG_ENCODER, "Encoder install isr service on same core failed");
        }
        attachedInterrupt = true;
    }
}

int32_t ESP32Encoder::getCount() {
    int16_t pcnt_count;
    int32_t correction = 0;
    _ENTER_CRITICAL();
    pcnt_get_counter_value(unit, &pcnt_count);
    // Check if counter overflowed, if so re-read and compensate.
    // See https://github.com/espressif/esp-idf/blob/v4.4.1/tools/unit-test-app/components/test_utils/ref_clock_impl_rmt_pcnt.c#L168-L172
    if (PCNT.int_st.val & BIT(unit)) {
        pcnt_get_counter_value(unit, &pcnt_count);
        if (PCNT.status_unit[unit].h_lim_lat) {
            correction = _INT16_MAX;
        } else if (PCNT.status_unit[unit].l_lim_lat) {
            correction = _INT16_MIN;
        }
    }
    _EXIT_CRITICAL();
    return overflow_count + correction + pcnt_count;
}

int32_t ESP32Encoder::getCountAndClear() {
    int16_t count;
    _ENTER_CRITICAL();
    pcnt_get_counter_value(unit, &count);
    pcnt_counter_clear(unit);
    _EXIT_CRITICAL();
    return count;
}

void ESP32Encoder::pause() {
    pcnt_counter_pause(unit);
}

void ESP32Encoder::resume() {
    pcnt_counter_resume(unit);
}

void ESP32Encoder::setFilter(uint16_t value) {
    if (value > 1023) {
        value = 1023;
    }
    if (value == 0) {
        pcnt_filter_disable(unit);
    } else {
        pcnt_set_filter_value(unit, value);
        pcnt_filter_enable(unit);
    }
}

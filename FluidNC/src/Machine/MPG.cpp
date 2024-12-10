// Copyright (c) 2025 -  Dumitru Petrusca
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.
//
// MPG handling. Also see LimitPin::trigger and Protocol::protocol_do_limit for MPG mode handling

#include "MPG.h"

#include "../Motors/MotorDriver.h"
#include "Axes.h"
#include "Float.h"
#include <Arduino.h>
#include <driver/pcnt.h>
#include "../Pin.h"

constexpr float          F            = 1000;  // 1000 ms/sec
static auto              ALPHA        = Float(1.0 / 10.0);
static const auto        BETA         = 1 - ALPHA;
static constexpr auto    MAX_MPG_RATE = 2.0f;    // Hz
static constexpr int32_t MAX_PULSE_DT = 500000;  // us
static portMUX_TYPE      MUX          = portMUX_INITIALIZER_UNLOCKED;

namespace Machine {
    void encoderISR(void* arg);

    void MPG::group(Configuration::HandlerBase& handler) {
        handler.item("a_pin", _aPin);
        handler.item("b_pin", _bPin);
        handler.item("reverse", _reverseTmp);
        handler.item("gain", _gainTmp);
        handler.item("pulses_per_rev", _pulsesPerRev);
        _reverse = Float(_reverseTmp ? -1.0 : 1.0);
        _maxGain = Float(_gainTmp);
    }

    void MPG::init(Axis* axis, const double isrRatePerSec) {
        _axis        = axis;
        _axisBitMask = bitnum_to_mask(_axis->getAxisNum());
        _isrRate     = Float(isrRatePerSec / F);  // step/ms

        // Compute the minimum time between MPG pulses
        const auto pps = 4 * _pulsesPerRev * MAX_MPG_RATE;  // pulses/sec
        _minPulseDt    = Float(1000000.0f / pps);           // us

        _pcntUnit = static_cast<pcnt_unit_t>(PCNT_UNIT_0 + _axis->getAxisNum());

        // Initialize Bresenham parameters
        _maxStepRate            = Float(_axis->maxStepRate() / F);                          // step/ms
        const auto acceleration = Float(_axis->_acceleration * axis->_stepsPerMm / F / F);  // (mm/ms^2) * (step/mm) = step/ms^2
        _oneOver2a              = 1 / (2 * acceleration);
        _stepRateIncrement      = acceleration / _isrRate;  // (step/ms^2) / (1/ms) = step/ms
        _currentStepRate        = 0;                        // motor starts at the bottom of the acceleration ramp
        _error                  = -_isrRate;
        _maxError               = 2 * _maxStepRate - _isrRate;  // max rate

        pcnt_config_t pcnt_config0 = {
            .pulse_gpio_num = _aPin.getNative(Pin::Capabilities::Input),  // rotary encoder A pin
            .ctrl_gpio_num  = _bPin.getNative(Pin::Capabilities::Input),  // rotary encoder B pin
            .lctrl_mode     = PCNT_MODE_KEEP,                             // B low
            .hctrl_mode     = PCNT_MODE_REVERSE,                          // B high
            .pos_mode       = PCNT_COUNT_INC,                             // A rising
            .neg_mode       = PCNT_COUNT_DEC,                             // A falling
            .counter_h_lim  = 30000,                                      // Set the counter upper limit to 30000
            .counter_l_lim  = -30000,                                     // Set the counter lower limit to -30000
            .unit           = _pcntUnit,                                  // Use PCNT unit
            .channel        = PCNT_CHANNEL_0,                             // Use PCNT channel 0
        };
        pcnt_unit_config(&pcnt_config0);

        pcnt_config_t pcnt_config1 = {
            .pulse_gpio_num = _bPin.getNative(Pin::Capabilities::Input),
            .ctrl_gpio_num  = _aPin.getNative(Pin::Capabilities::Input),
            .lctrl_mode     = PCNT_MODE_REVERSE,  // A low
            .hctrl_mode     = PCNT_MODE_KEEP,     // A high
            .pos_mode       = PCNT_COUNT_INC,     // B rising
            .neg_mode       = PCNT_COUNT_DEC,     // B falling
            .counter_h_lim  = 30000,
            .counter_l_lim  = -30000,
            .unit           = _pcntUnit,
            .channel        = PCNT_CHANNEL_1,
        };
        pcnt_unit_config(&pcnt_config1);
        pcnt_set_filter_value(_pcntUnit, 1000);                 // Set the filter value to 1000 * 12.5ns = 12.5us filter
        pcnt_filter_enable(_pcntUnit);                          // Enable the filter
        pcnt_event_enable(_pcntUnit, PCNT_EVT_THRES_0);         // Enable event for threshold 0
        pcnt_set_event_value(_pcntUnit, PCNT_EVT_THRES_0, 1);   // Trigger ISR on count +1
        pcnt_event_enable(_pcntUnit, PCNT_EVT_THRES_1);         // Enable event for threshold 1
        pcnt_set_event_value(_pcntUnit, PCNT_EVT_THRES_1, -1);  // Trigger ISR on count -1
        pcnt_counter_pause(_pcntUnit);                          // Pause the counter
        pcnt_counter_clear(_pcntUnit);                          // Clear the counter
        pcnt_isr_service_install(0);                            // Install the ISR service
        pcnt_isr_handler_add(_pcntUnit, encoderISR, this);      // Register the ISR handler
        pcnt_counter_resume(_pcntUnit);                         // Start the counter

        log_info(" MPG A:" << _aPin << " B:" << _bPin << " Gain: " << _maxGain.toFloat());
        log_info("  ISR rate (step/s)       :" << isrRatePerSec);
        log_info("  ISR rate (step/ms)      :" << _isrRate.toFloat());
        log_info("  Max Step Rate (step/ms) :" << _maxStepRate.toFloat());
        log_info("  Acceleration (step/ms^2):" << acceleration.toFloat());
        log_info("  1/2a                    :" << _oneOver2a.toFloat());
        log_info("  Rate Increment (step/ms):" << _stepRateIncrement.toFloat());
        log_info("  Acceleration Steps      :" << (_oneOver2a * _maxStepRate * _maxStepRate).toFloat());
        log_info("  Min Pulse Period (us)   :" << _minPulseDt.toFloat());
    }

    void IRAM_ATTR encoderISR(void* arg) {
        auto* mpg = static_cast<MPG*>(arg);
        // get the count and clear the counter
        int16_t count;
        pcnt_get_counter_value(mpg->_pcntUnit, &count);
        pcnt_counter_clear(mpg->_pcntUnit);
        if (count == 0) {
            return;
        }
        // measure time since last pulse
        const auto time     = esp_timer_get_time();                                // us
        const auto dt       = min(time - mpg->_lastPulseTime, i64(MAX_PULSE_DT));  // us
        mpg->_lastPulseTime = time;
        // compute the average pulse interval in us
        mpg->_pulseDt = ALPHA * i32(dt / abs(count)) + BETA * mpg->_pulseDt;
        // compute the gain
        const auto x    = mpg->_minPulseDt / mpg->_pulseDt;
        const auto gain = min(1 + (mpg->_maxGain - 1) * x, mpg->_maxGain);
        // compute step count increment
        const auto totalStepIncrement = gain * count * mpg->_reverse;
        // quit if we're moving into a triggered limit
        const auto posLimitHit        = bits_are_true(Machine::Axes::posLimitMask, mpg->_axisBitMask);
        const auto negLimitHit        = bits_are_true(Machine::Axes::negLimitMask, mpg->_axisBitMask);
        const auto movingIntoPosLimit = posLimitHit && totalStepIncrement > 0;
        const auto movingIntoNegLimit = negLimitHit && totalStepIncrement < 0;
        if (movingIntoPosLimit || movingIntoNegLimit) {
            // The system will not enter the alarm state or limit the motors
            // See LimitPin::trigger and Protocol::protocol_do_limit for MPG mode handling
            return;
        }
        mpg->incrementStepCount(totalStepIncrement);
    }

    Float IRAM_ATTR MPG::getStepDirection() const {
        if (_locked) {
            return 0;
        }
        // Snapshot _step_count atomically to avoid race conditions
        const auto currentStepCount = getStepCount();
        if (currentStepCount >= 1) {
            return 1;
        }
        if (currentStepCount <= -1) {
            return -1;
        }
        return 0;
    }

    int32_t IRAM_ATTR MPG::getStep() {
        // Snapshot the step count atomically to avoid race conditions
        const auto currentStepCount = getStepCount();

        // compute the number of steps required to ramp down from the current rate
        const auto rampDownStepCount = _oneOver2a * _currentStepRate * _currentStepRate;
        // accelerate or decelerate accordingly
        if (currentStepCount.abs() > rampDownStepCount) {
            _currentStepRate += _stepRateIncrement;  // accelerate
            if (_currentStepRate > _maxStepRate) {
                _currentStepRate = _maxStepRate;
            }
        } else {
            _currentStepRate -= _stepRateIncrement;  // decelerate
            if (_currentStepRate < 0) {
                _currentStepRate = 0;
            }
        }

        // Emit steps using Bresenham algorithm
        _error += _currentStepRate * 2;  // advance the Bresenham algorithm
        const auto direction = getStepDirection();
        if (direction != 0) {            // there is a step to make
            if (_error > 0) {            // are we allowed to make the step now?
                _error -= _isrRate * 2;  // deduct the "cost" of the step
                incrementStepCount(-direction);
                return direction.toInt();
            }
        } else if (_error > _maxError) {
            _error = _maxError;  // cap the error
        }
        return 0;
    }

    Float IRAM_ATTR MPG::getStepCount() const {
        portENTER_CRITICAL_ISR(&MUX);
        const auto currentStepCount = _stepCount;
        portEXIT_CRITICAL_ISR(&MUX);
        return currentStepCount;
    }

    void IRAM_ATTR MPG::incrementStepCount(const Float inc) {
        portENTER_CRITICAL_ISR(&MUX);
        _stepCount += inc;
        portEXIT_CRITICAL_ISR(&MUX);
    }

    void MPG::reset() {
        portENTER_CRITICAL_ISR(&MUX);
        _stepCount = 0;
        portEXIT_CRITICAL_ISR(&MUX);
    }

    void MPG::toggleLocked() {
        reset();
        _locked = !_locked;
    }
}

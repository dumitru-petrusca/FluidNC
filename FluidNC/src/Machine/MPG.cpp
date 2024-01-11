// Copyright (c) 2021 -  Stefan de Bruijn
// Copyright (c) 2021 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "MPG.h"

#include "../Motors/MotorDriver.h"
#include "Axes.h"
#include "Driver/fluidnc_gpio.h"

namespace Machine {

    void MPG::group(Configuration::HandlerBase& handler) {
        handler.item("a_pin", _aPin);
        handler.item("b_pin", _bPin);
        handler.item("reverse", _reverse);
        handler.item("pulses_per_rev", _pulses_per_rev);
        handler.item("max_rpm", _max_rpm);
        _max_rpm_fp = FP(_max_rpm);
        handler.item("max_rate_factor", _max_rate_factor);
    }

    void MPG::afterParse() {}

    bool IRAM_ATTR redundantRead(Pin* pin) {
        int sum = 0;
        for (int i = 0; i < 5; i++) {
            sum += pin->read(); // TODO-dp maybe add some delay here
        }
        return sum >= 3;
    }

    void IRAM_ATTR encoderISR(void* arg) {
        auto*    mpg                    = (MPG*)arg;
        uint64_t time_us                = micros();
        uint64_t time_since_last_isr_us = time_us - mpg->_last_isr_time_us;
        mpg->_last_isr_time_us          = time_us;
        // check if this is a real pulse
        if (time_since_last_isr_us >= mpg->_refractory_period_us && redundantRead(&mpg->_aPin) == 1) {
            mpg->_duration_micros    = time_us - mpg->_last_pulse_time_us;
            mpg->_last_pulse_time_us = time_us;
            int8_t direction         = redundantRead(&mpg->_bPin) == 1 ? -1 : +1;  // 1 = CW, 0 = CCW
            mpg->_count += direction;
        }
    }

    void MPG::init(Axis* axis, uint32_t maxStepRateForAllAxes) {
        log_info(" MPG A:" << _aPin << " B:" << _bPin);
        _axis = axis;

        // Assuming the hand wheel is rotated at maximum of 240 rpm and a 500 ppr encoder.
        // 240 rpm = 4 rev/sec
        // 4 rev/sec * 500 pulse/rev = 2000 pulse/sec
        // 10^6 ms / 2000 = 500 us between pulses
        double pulses_per_second = (_max_rpm / 60.0) * _pulses_per_rev;
        float  refractory_factor = 0.5;
        _refractory_period_us    = 1e6 * refractory_factor / pulses_per_second;

        pinnum_t aPinNum = _aPin.getNative(Pin::Capabilities::Input);
        pinnum_t bPinNum = _bPin.getNative(Pin::Capabilities::Input);
        gpio_reset_pin(static_cast<gpio_num_t>(aPinNum));
        gpio_reset_pin(static_cast<gpio_num_t>(bPinNum));
        gpio_mode(aPinNum, true, false, false, false, false);
        gpio_mode(bPinNum, true, false, false, false, false);
        attachInterruptArg(digitalPinToInterrupt(aPinNum), encoderISR, this, RISING);

        float maxStepRate      = axis->_stepsPerMm * axis->_maxRate * _max_rate_factor / 60.0;
        float maxPulseRate     = _pulses_per_rev * _max_rpm / 60;
        _maxPulseMultiplier_fp = FP(maxStepRate / maxPulseRate);

        _t = 100;
        _n = (int)(maxStepRate / maxStepRateForAllAxes * _t);
        _e = 2 * _n - _t;
    }

    int32_t IRAM_ATTR MPG::getCountDelta() {
        int32_t count = _count;
        int32_t dc    = count - _old_count;
        _old_count    = count;
        return dc;
    }

    void IRAM_ATTR MPG::reset() {
        _old_count = 0;
        _count     = 0;
    }

    int32_t IRAM_ATTR MPG::getRpm() const {
        return 60 * 1e6 / _duration_micros / _pulses_per_rev;
    }

    int32_t IRAM_ATTR MPG::countToSteps(int32_t counts) {
        fixed_t rel_rpm = DIV(FP(getRpm()), _max_rpm_fp);                    // compute relative rpm
        rel_rpm         = MIN(ONE, rel_rpm);                                 // cap relative rpm to 1
        fixed_t m       = MUL(_maxPulseMultiplier_fp - ONE, rel_rpm) + ONE;  // m = (maxPulseMultiplier - 1) * rel_rpm + 1
        return INT(MUL(FP(counts), m));                                      // r = counts * m
    }

    int32_t IRAM_ATTR MPG::get_pending_steps() {
        // MPG locked, return
        if (_locked) {
            return 0;
        }

        // Advance Bresenham algorithm and see if we should emit a step
        if (_e > 0) {
            _e += 2 * _n - 2 * _t;
        } else {
            _e += 2 * _n;
            return 0;  // No step should be emitted, return
        }

        // Update the encoder count and compute the delta
        _target_steps += countToSteps(getCountDelta());
        int32_t delta = _target_steps - _current_steps;

        // check limits and stop motion in the direction that triggers the limit switch
        int axisBitMask = bitnum_to_mask(_axis->getAxisNum());
        if ((bits_are_true(Machine::Axes::posLimitMask, axisBitMask) && delta > 0) ||
            (bits_are_true(Machine::Axes::negLimitMask, axisBitMask) && delta < 0)) {
            reset();
            _target_steps = _current_steps;
            delta         = 0;
        }

        return delta;
    }

    void IRAM_ATTR MPG::update_pending_steps(int8_t inc) {
        _current_steps += inc;
    }

    void MPG::toggleLocked() {
        reset();
        _locked = !_locked;
    }

}

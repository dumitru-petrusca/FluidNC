// Copyright (c) 2021 -  Stefan de Bruijn
// Copyright (c) 2021 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Configuration/Configurable.h"
#include "LimitPin.h"
#include "Axis.h"
#include <Arduino.h>
#include "src/Pins/PinDetail.h"

#define fixed_t int32_t

#define FP(x) ((x)*100)
#define ONE FP(1)

#define INT(x) ((x) / 100)
#define MUL(x, y) ((x) * (y) / 100)
#define DIV(x, y) (((x)*100) / (y))

namespace Machine {
    class Axis;

    class MPG : public Configuration::Configurable {
    public:
        MPG() = default;
        void    init(Axis* axis, uint32_t maxStepRateForAllAxes);
        void    toggleLocked();
        int32_t get_pending_steps();
        void    update_pending_steps(int8_t inc);
        void    reset();

        Axis*    _axis;
        Pin      _aPin;
        Pin      _bPin;
        bool     _reverse = false;
        bool     _locked  = false;
        uint32_t _pulses_per_rev;
        float    _max_rpm;
        fixed_t  _max_rpm_fp;
        float    _max_rate_factor;
        fixed_t  _maxPulseMultiplier_fp;

        uint32_t _refractory_period_us;
        volatile uint64_t _last_isr_time_us   = 0;
        volatile uint64_t _last_pulse_time_us = 0;
        volatile uint64_t _duration_micros    = 0;
        volatile int32_t  _count              = 0;
        volatile int32_t  _old_count          = 0;

    private:
        // in ISR implementation
        volatile int32_t _target_steps  = 0;
        volatile int32_t _current_steps = 0;

        // Bresenham params
        int32_t _n = 0;
        int32_t _t = 0;
        int32_t _e = 0;

        // Configuration system helpers
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;

        int32_t getCountDelta();
        int32_t getRpm() const;
        int32_t countToSteps(int32_t counts);
    };

}

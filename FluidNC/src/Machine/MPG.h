// Copyright (c) 2025 -  Dumitru Petrusca
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "../Configuration/Configurable.h"
#include "Axis.h"
#include "Float.h"
#include <driver/pcnt.h>

namespace Machine {
    class Axis;

    class MPG : public Configuration::Configurable {
    public:
        MPG() = default;
        void                init(Axis* axis, double isrRatePerSec);
        void                incrementStepCount(Float inc);
        void                toggleLocked();
        int32_t             getStep();
        [[nodiscard]] Float getStepCount() const;
        void                reset();

        Axis*    _axis;
        uint32_t _axisBitMask = 0;

        Pin   _aPin;
        Pin   _bPin;
        float _pulsesPerRev = 500;
        bool  _reverseTmp   = false;
        Float _reverse      = 1;
        float _gainTmp      = 1;

        // MPG state
        bool        _locked        = false;
        pcnt_unit_t _pcntUnit      = PCNT_UNIT_0;
        Float       _maxGain       = 1;
        int64_t     _lastPulseTime = 0;
        Float       _pulseDt       = 1;
        Float       _minPulseDt    = 1;

        // Bresenham state
        Float _stepCount         = 0;
        Float _isrRate           = 1;
        Float _maxStepRate       = 1;
        Float _currentStepRate   = 1;
        Float _stepRateIncrement = 0;
        Float _oneOver2a         = 0;
        Float _error             = 0;
        Float _maxError          = 0;

    private:
        [[nodiscard]] Float getStepDirection() const;

        // Configuration system helpers
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override {}
    };

}

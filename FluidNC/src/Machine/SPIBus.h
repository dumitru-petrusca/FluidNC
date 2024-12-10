// Copyright (c) 2021 -  Stefan de Bruijn
// Copyright (c) 2021 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "driver/spi_common.h"

namespace Machine {
    class SPIBus : public Configuration::Configurable {
    public:
        SPIBus() = default;

        Pin _miso;
        Pin _mosi;
        Pin _sck;
        spi_host_device_t _host_id = HSPI_HOST;

        void validate() override;
        void group(Configuration::HandlerBase& handler) override;
        void afterParse() override;

        void init();
        void deinit();

        bool defined();

        ~SPIBus() = default;

    private:
        bool _defined = false;
    };
}

// Copyright (c) 2025 -	Dumitru Petrusca
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Configuration/Configurable.h"
#include "Pin.h"
#include "Error.h"
#include <cstdint>

#define RETURN_IF_ERROR(expr)               \
    do {                                    \
        int __err = (expr);                 \
        if (__err != ESP_OK) return __err;  \
    } while (0)

class W5500 : public Configuration::Configurable {
public:
    Pin _interrupt;
    Pin _cs;
    std::string _ip;
    std::string _gateway;
    std::string _netmask;
    esp_err_t _err = ESP_ERR_INVALID_STATE;
   
    W5500() {};
    W5500(const W5500&)            = delete;
    W5500& operator=(const W5500&) = delete;
    ~W5500() {};

    void init();
    bool started();

    void group(Configuration::HandlerBase& handler) override {
       handler.item("cs_pin", _cs);
       handler.item("interrupt_pin", _interrupt);
       handler.item("ip", _ip);
       handler.item("gateway", _gateway);
       handler.item("netmask", _netmask);
    }

    void afterParse() override;

private:
    esp_err_t setupW5500();
};

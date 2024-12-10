
#pragma once

#include <OLEDDisplay.h>
#include "Machine/I2CBus.h"
#include <algorithm>

using namespace Machine;

class SH1106_I2C : public OLEDDisplay {
private:
    uint8_t _address;
    I2CBus* _i2c;
    int     _frequency;
    bool    _error = false;

public:
    SH1106_I2C(uint8_t address, OLEDDISPLAY_GEOMETRY g, I2CBus* i2c, int frequency) :
        _address(address), _i2c(i2c), _frequency(frequency), _error(false) {
        setGeometry(g);
    }

    bool connect() { return true; }

    void display(void) {
        if (_error) {
            return;
        }
        const int x_offset  = (132 - this->width()) / 2;
        uint8_t   minBoundY = UINT8_MAX;
        uint8_t   maxBoundY = 0;

        uint8_t minBoundX = UINT8_MAX;
        uint8_t maxBoundX = 0;
        uint8_t x, y;

        // Calculate the Y bounding box of changes
        // and copy buffer[pos] to buffer_back[pos];
        for (y = 0; y < (this->height() / 8); y++) {
            for (x = 0; x < this->width(); x++) {
                uint16_t pos = x + y * this->width();
                if (buffer[pos] != buffer_back[pos]) {
                    minBoundY = std::min(minBoundY, y);
                    maxBoundY = std::max(maxBoundY, y);
                    minBoundX = std::min(minBoundX, x);
                    maxBoundX = std::max(maxBoundX, x);
                }
                buffer_back[pos] = buffer[pos];
            }
            yield();
        }

        // If the minBoundY wasn't updated
        // we can safely assume that buffer_back[pos] == buffer[pos]
        // holds true for all values of pos
        if (minBoundY == UINT8_MAX)
            return;

        // Calculate the colum offset
        uint8_t minBoundXp2H = (x_offset + minBoundX) & 0x0F;
        uint8_t minBoundXp2L = 0x10 | ((x_offset + minBoundX) >> 4);

        for (y = minBoundY; y <= maxBoundY; y++) {
            sendCommand(0xB0 + y);
            sendCommand(minBoundXp2H);
            sendCommand(minBoundXp2L);

            uint8_t* start = &buffer[(minBoundX + y * this->width()) - 1];
            uint8_t  save  = *start;
            *start         = 0x40;  // control
            _i2c->write(_address, start, (maxBoundX - minBoundX) + 1 + 1);
            *start = save;
        }
    }

private:
    int getBufferOffset(void) { return 0; }

    inline void sendCommand(uint8_t command) __attribute__((always_inline)) {
        if (_error) {
            return;
        }
        uint8_t _data[2];
        _data[0] = 0x80;  // control
        _data[1] = command;
        if (_i2c->write(_address, _data, sizeof(_data)) < 0) {
            log_error("OLED is not responding");
            _error = true;
        }
    }
};

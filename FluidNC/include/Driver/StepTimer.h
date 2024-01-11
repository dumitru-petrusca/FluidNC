#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

void stepTimerInit(uint32_t frequency, bool (*fn)(void));
void IRAM_ATTR stepTimerStart(uint64_t alarm);
void stepTimerStop();
void stepTimerSetTicks(uint32_t ticks);

#ifdef __cplusplus
}
#endif

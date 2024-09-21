#pragma once
#include <stdint.h>

void initializeTimer(uint8_t prescaler);
#define getTimerCount() ((uint16_t)TCNT1)

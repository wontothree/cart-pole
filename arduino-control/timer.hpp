#ifndef TIMER_HPP
#define TIMER_HPP

#include <avr/io.h>

void initializeTimer(uint8_t prescaler);

#define getTimerCount() ((uint16_t)TCNT1)

#endif

#ifndef TIMER_HPP
#define TIMER_HPP

#include <avr/io.h>

void initializeTimer(uint8_t prescaler);

uint16_t getTimerCount();

#endif

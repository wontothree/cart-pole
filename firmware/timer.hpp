#ifndef TIMER_HPP
#define TIMER_HPP

#include <avr/io.h>

void initialize_timer1(uint8_t prescaler);
uint16_t get_timer1_count();

#endif

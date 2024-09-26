#include "timer.hpp"
#include <avr/interrupt.h>

void initializeTimer(uint8_t prescaler)
{
  // disable interrupts
  cli();

  // initialize timer1 setting
  TCCR1A = 0; // initialize register TCCR1A
  TCCR1B = 0; // initialize register TCCR1B
  TCNT1 = 0;  // initialize Timer 1 count

  // prescaler mode
  if (prescaler == 1)
  {
    TCCR1B |= (1 << CS10); // 1 x prescaler
  }
  else if (prescaler == 8)
  {
    TCCR1B |= (1 << CS11); // 8 x prescaler
  }
  else if (prescaler == 64)
  {
    TCCR1B |= (1 << CS11) | (1 << CS10); // 64 x prescaler
  }

  // enable interrupts
  sei();
}
#ifndef STEPPER_
#define STEPPER_HPP

#include "uart.hpp" // 전역 변수
#include <Arduino.h> // For digitalWrite and pinMode

// define pin numbers
#define PIN_A 10
#define PIN_NA 11
#define PIN_B 12
#define PIN_NB 13

extern volatile int16_t stepper_motor_tick;

// voltage pattern to be applied to each coil of a stepper motor (A, B, A_, B_)
extern int step_info[8][4];

// apply voltage to each coils A, B, A_, B_
void moveOneStep();

#endif // STEPPER_HPP

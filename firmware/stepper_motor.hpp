#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <Arduino.h> // For digitalWrite and pinMode

#include "globals.hpp"

extern volatile int16_t stepper_motor_tick;

// voltage pattern to be applied to each coil of a stepper motor (A, B, A_, B_)
extern int step_info[8][4];

// apply voltage to each coils A, B, A_, B_
void moveOneStep();

#endif // STEPPER_HPP

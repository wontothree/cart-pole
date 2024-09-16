#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <stdint.h>
#include <avr/interrupt.h>  // for `noInterrupts()` and `interrupts()`
#include <Arduino.h>        // For digitalWrite and pinMode

#include "globals.hpp"

extern volatile int16_t stepper_motor_tick;

// voltage pattern to be applied to each coil of a stepper motor (A, B, A_, B_)
extern int step_info[8][4];

// apply voltage to each coils A, B, A_, B_
void moveOneStep();


void initialize_motor_pins();

void control_motor(uint16_t current_count, float& current_velocity, uint16_t& last_step_count, uint16_t& last_control_count, uint16_t& step_interval_counts);

#endif // STEPPER_HPP

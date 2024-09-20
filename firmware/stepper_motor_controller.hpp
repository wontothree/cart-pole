#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <stdint.h>
#include <avr/interrupt.h>  // for `noInterrupts()` and `interrupts()`
#include <Arduino.h>        // For digitalWrite and pinMode

#include "globals.hpp"

extern volatile uint16_t stepper_motor_tick;

// voltage pattern to be applied to each coil of a stepper motor (A, B, A_, B_)
extern int step_info[8][4];

// apply voltage to each coils A, B, A_, B_
void moveOneStep();

void initialize_motor_pins();

void update_motor_control(uint16_t current_count, uint16_t& step_interval_counts, float& current_linear_velocity, float target_velocity);

void update_motor_control_by_accel(uint16_t current_count, uint16_t& target_step_interval_counts, float& current_linear_velocity, float target_current_linear_acceleration);

void handle_motor_direction();

#endif // STEPPER_HPP

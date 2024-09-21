#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <stdint.h>
#include <avr/interrupt.h>  // for `noInterrupts()` and `interrupts()`
#include <Arduino.h>        // For digitalWrite and pinMode

#include "globals.hpp"

extern volatile uint16_t stepper_motor_tick;

// voltage pattern to be applied to each coil of a stepper motor (A, B, A_, B_)
extern int step_info[8][4];

// float currentPosition = 0;
// float currentVelocity = 0;

// uint16_t lastMotorUpdateCount = 0;
// uint16_t currentMotorInterval = 9999;

// apply voltage to each coils A, B, A_, B_
void moveOneStep();

void initializeStepperMotorPins();

void update_motor_control_by_accel(uint16_t current_count, uint16_t& target_step_interval_counts, float& current_linear_velocity, float target_current_linear_acceleration);

// /*
//  * 모터를 가속도에 따라 제어하고, 현재 속도와 틱을 업데이트한다.
//  * @param currentCounter
//  * @param acceleration
//  */
// void updateMotorByAcceleration(uint16_t currentCounter, float acceleration);

#endif // STEPPER_HPP

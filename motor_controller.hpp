#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <stdint.h>
#include <Arduino.h>

#include "globals.hpp"
#include "stepper_motor.hpp"

// 모터 제어 함수 선언
void initialize_motor();
void control_motor(uint16_t current_count, float& current_velocity, uint16_t& last_step_count, uint16_t& last_control_count, uint16_t& step_interval_counts);

// // 전역 변수 선언
// extern volatile int direction;
// extern volatile bool isDirectionChanged;
// extern float target_velcocity;

#endif // MOTOR_CONTROLLER_HPP

#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <stdint.h> // for uint16_t

// define pin numbers
#define PIN_A 10
#define PIN_NA 11
#define PIN_B 12
#define PIN_NB 13

// global variables
extern volatile int direction;                  // 1 : clockwise, -1 : counter-clockwise
extern volatile bool isDirectionChanged;

extern volatile float target_velocity;
extern volatile float target_current_linear_acceleration;

extern volatile uint16_t stepper_motor_tick;
extern volatile uint16_t stepper_motor_tick_observation_time; 

extern uint16_t currentMotorInterval;   // 모터 간격
extern uint16_t lastMotorUpdateCount;   // 마지막 업데이트 카운트
extern float acceleration;              // 가속도

extern float currentVelocity;           // 현재 속도
extern float currentPosition;           // 현재 위치

#endif // GLOBALS_HPP

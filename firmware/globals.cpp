#include "globals.hpp"

volatile int direction = 1;      
volatile bool isDirectionChanged = false;       
volatile float target_velocity = 1;  

volatile int16_t stepper_motor_tick = 0;
volatile unsigned long stepper_motor_tick_observation_time = 0;
#include "globals.hpp"

volatile int direction = 1;      
volatile bool isDirectionChanged = false;     
  
volatile float target_velocity = 1;  
volatile float target_current_linear_acceleration = 1;

volatile uint16_t stepper_motor_tick = 0;
volatile uint16_t stepper_motor_tick_observation_time = 0;

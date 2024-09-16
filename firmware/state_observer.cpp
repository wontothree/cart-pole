// #include "state_observer.hpp"
// #include <Arduino.h> // Arduino 함수 및 상수 사용을 위한 헤더

// const float STEPS_PER_METER = 6366 // 6366.198

// static unsigned long previous_time = 0;
// static int16_t previous_tick = 0;

// void calculate_cart_position_and_velocity(int16_t stepper_motor_tick, unsigned long stepper_motor_tick_observation_time) {
//     // Check if delta_time is non-zero to avoid division by zero
//     if (stepper_motor_tick_observation_time - previous_time > 0) {
//         float cart_position = stepper_motor_tick / STEPS_PER_METER; // m

//         float cart_velocity = ((stepper_motor_tick - previous_tick) / STEPS_PER_METER) / ((stepper_motor_tick_observation_time - previous_time) / 1000.0); // m/s

//         // // Output or use the results as needed
//         // Serial.print("Cart Position: ");
//         // Serial.print(cart_position, 2); // Print with 2 decimal places
//         // Serial.print(" meters, Cart Velocity: ");
//         // Serial.print(cart_velocity, 2); // Print with 2 decimal places
//         // Serial.println(" meters/second");

//         previous_time = stepper_motor_tick_observation_time;
//         previous_tick = stepper_motor_tick;

//         return cart_position, cart_velocity
//     }
// }

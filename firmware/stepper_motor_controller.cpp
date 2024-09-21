#include "stepper_motor_controller.hpp"

// 1, 2 phase excitation method
int step_info[8][4] = 
{
  { HIGH, LOW, LOW, LOW }, // one phase, 0.9 degree, or 1/400 revolution
  { HIGH, HIGH, LOW, LOW },
  { LOW, HIGH, LOW, LOW },
  { LOW, HIGH, HIGH, LOW },
  { LOW, LOW, HIGH, LOW },
  { LOW, LOW, HIGH, HIGH },
  { LOW, LOW, LOW, HIGH },
  { HIGH, LOW, LOW, HIGH }
};

// rotate one phase (0.9 degree)
void moveOneStep()
{
    static int step = 0;
    digitalWrite(PIN_A, step_info[step][0]);
    digitalWrite(PIN_B, step_info[step][1]);
    digitalWrite(PIN_NA, step_info[step][2]);
    digitalWrite(PIN_NB, step_info[step][3]);
    step += direction;
    if (step > 7) step = 0;
    if (step < 0) step = 7;

    // record stepper motor tick for obtaining position of cart
    stepper_motor_tick += direction;

    // record stepper motor tick observation time for obtaining velocity of cart
    stepper_motor_tick_observation_time = millis();
}

// pin map
void initializeStepperMotorPins()
{
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_NA, OUTPUT);
    pinMode(PIN_NB, OUTPUT);
}

void update_motor_control_by_accel(uint16_t current_count, uint16_t& target_step_interval_counts, float& current_linear_velocity, float target_current_linear_acceleration)
{
    const uint16_t motor_control_period = 0.01;
    const uint16_t motor_control_counts = motor_control_period * 16000000 / 64;
    
    static uint32_t last_motor_control_count = 0;
    static uint32_t last_step_count = 0;

    static uint32_t last_motor_control_count2 = 0;

    // update target next velocity
    // float target_next_linear_velocity = current_linear_velocity + motor_control_period * target_current_linear_acceleration;
    // float target_next_linear_velocity = target_current_linear_acceleration;

    float target_next_linear_velocity = current_linear_velocity; // motor_control_period * target_current_linear_acceleration;

    // calculate target step interval counts
    if ((current_count - last_motor_control_count) >= motor_control_counts) {
        target_step_interval_counts = (uint16_t)(200.f / target_next_linear_velocity); 

        last_motor_control_count = current_count;
    }
    
    // move one step
    if ((current_count - last_step_count) >= target_step_interval_counts) {
        moveOneStep();

        last_step_count = current_count;
    }
}

// void updateMotorByAcceleration(uint16_t currentCount, float acceleration)
// {
//     const float TICK_PER_METER = 6366.197;   // from 400 tick = 2 pi (0.01m)
//     const float COUNT_PER_SECOND = 0.000004; // from 1 count = 1/(16M/64) s

//     // move one step 
//     if (currentCount - lastMotorUpdateCount >= currentMotorInterval)
//     {
//         lastMotorUpdateCount += currentMotorInterval;
//         moveOneStep();

//         float newMotorInterval = ((float) currentMotorInterval / (1 + acceleration * currentMotorInterval)) * TICK_PER_METER / COUNT_PER_SECOND;
        
//         currentMotorInterval = (uint16_t) newMotorInterval;

//         // update position of cart (m) (when tick = 1)
//         currentPosition += 1 / TICK_PER_METER;

//         // update velocity of cart (m/s)
//         // from (count/s) / (tick/m * count) = m/s (when tick = 1)
//         currentVelocity = COUNT_PER_SECOND / (TICK_PER_METER * (float) currentMotorInterval);
//     }
// }

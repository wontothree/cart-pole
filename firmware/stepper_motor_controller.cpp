#include "stepper_motor_controller.hpp"

// 1, 2 phase excitation method
int step_info[8][4] = {
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
void moveOneStep() {
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
void initialize_motor_pins() {
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_NA, OUTPUT);
    pinMode(PIN_NB, OUTPUT);
}

// motor control
void update_motor_control(uint16_t current_count, uint16_t& step_interval_counts, float& current_velocity, float target_velocity) {
    static uint16_t last_step_count = 0;
    static uint16_t last_control_count = 0;

    if ((current_count - last_step_count) > step_interval_counts) {
        moveOneStep();
        last_step_count = current_count;
    }

    if ((current_count - last_control_count) > MOTOR_CONTROL_COUNTS) {
        if (current_velocity < target_velocity) {
            current_velocity += 0.0005f;
        } else if (current_velocity > target_velocity) {
            current_velocity -= 0.0005f;
        }

        step_interval_counts = (uint16_t)(80.f / current_velocity); // 314
        
        last_control_count = current_count;
    }
}

void handle_motor_direction() {
    if (isDirectionChanged) {
        noInterrupts();
        direction = (direction == 1) ? -1 : 1;
        isDirectionChanged = false;
        interrupts();
    }
}

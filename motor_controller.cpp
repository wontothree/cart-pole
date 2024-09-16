#include "motor_controller.hpp"
#include <avr/interrupt.h>  // for `noInterrupts()` and `interrupts()`

// // 전역 변수 정의
// volatile int direction = 1;        // 1 : 시계방향, -1 : 반시계방향
// volatile bool isDirectionChanged = false;
// float target_velcocity = 1;

void initialize_motor() {
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_NA, OUTPUT);
    pinMode(PIN_NB, OUTPUT);
}

// 모터 제어 함수
void control_motor(uint16_t current_count, float& current_velocity, uint16_t& last_step_count, uint16_t& last_control_count, uint16_t& step_interval_counts) {
    const uint16_t MOTOR_CONTROL_COUNTS = 200;

    // 스텝 모터 제어
    if ((current_count - last_step_count) > step_interval_counts) {
        moveOneStep();  // 스텝 이동
        last_step_count = current_count;
    }

    if ((current_count - last_control_count) > MOTOR_CONTROL_COUNTS) {
        if (current_velocity < target_velcocity) {
            current_velocity += 0.0005f;
        } else if (current_velocity > target_velcocity) {
            current_velocity -= 0.0005f;
        }

        step_interval_counts = (uint16_t)(314.f / current_velocity);
        last_control_count = current_count;
    }

    // 방향 설정
    if (isDirectionChanged) {
        noInterrupts();
        direction = (direction == 1) ? -1 : 1;
        isDirectionChanged = false;
        interrupts();
    }
}

#include "stepping_motor.hpp"
#include "uart.hpp"

StepperMotor motor;
volatile bool isDirectionChanged = false;
float target_velocity = 1;

ISR(USART_RX_vect) {
    uint8_t receivedByte = UDR0;
    if ((receivedByte >= '0') && (receivedByte <= '9')) {
        target_velocity = (receivedByte - '0') * 0.4f;
        motor.setTargetVelocity(target_velocity);
    } else if (receivedByte == ' ') { // 스페이스바 입력
        isDirectionChanged = true; // 방향 전환 플래그 설정
    }
}

void setup() {
    init_UART(9600); 
    motor.initialize();

    // disable interrupts
    noInterrupts();

    // set timer (Timer1)
    TCCR1A = 0;  // initialize TCCR1A register
    TCCR1B = 0;  // initialize TCCR1B register
    TCNT1 = 0;   // set timer 1 count to 0
    TCCR1B = (1 << CS11); // 8 분주

    // enable interrupts
    interrupts();

    // core logic
    float current_velocity = 1;
    uint16_t last_step_count = 0;
    uint16_t last_control_count = 0;
    uint16_t step_interval_counts = 314;
    const uint16_t MOTOR_CONTROL_COUNTS = 200;

    while (true) {
        // clock count
        uint16_t current_count = (uint16_t)TCNT1;

        // step motor
        if ((current_count - last_step_count) > step_interval_counts) {
            motor.moveOneStep();
            last_step_count = current_count;
        }

        // set acceleration
        if ((current_count - last_control_count) > MOTOR_CONTROL_COUNTS) {
            if (current_velocity < motor.target_velocity) {
                current_velocity += 0.0005f;
            } else if (current_velocity > motor.target_velocity) {
                current_velocity -= 0.0005f;
            }

            step_interval_counts = (uint16_t)(314.f / current_velocity);
            
            last_control_count = current_count;
        }

        // set direction
        if (isDirectionChanged) {
            noInterrupts();
            delay(50);  // 잠시 멈추고
            motor.setDirection((motor.direction == 1) ? -1 : 1);
            isDirectionChanged = false;
            interrupts();
        }
    }
}

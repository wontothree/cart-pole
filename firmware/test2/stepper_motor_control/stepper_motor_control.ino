#include "uart.hpp"
#include "stepper_motor.hpp"

void setup() {
  initialize_uart(9600); 

  // set stepper motor pins as outputs
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_NA, OUTPUT);
  pinMode(PIN_NB, OUTPUT);

  // disable interrupts
  noInterrupts();

  // set timer (Timer1)
  TCCR1A = 0;  // initialize TCCR1A register
  TCCR1B = 0;  // initialize TCCR1B register
  TCNT1 = 0;   // set timer 1 count to 0
  // TCCR1B |= (1 << CS11) | (1 << CS10); // 64분주 mode
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
      moveOneStep();
      last_step_count = current_count;
    }

    // set motor
    if ((current_count - last_control_count) > MOTOR_CONTROL_COUNTS) {
      if (current_velocity < target_velcocity) {
        current_velocity += 0.0005f;
      } else if (current_velocity > target_velcocity) {
        current_velocity -= 0.0005f;
      }

      step_interval_counts = (uint16_t)(314.f / current_velocity);
      
      last_control_count = current_count;
    }

    // set direction
    if (isDirectionChanged) {
      noInterrupts();
      delay(50);  // 잠시 멈추고
      direction = (direction == 1) ? -1 : 1;
      isDirectionChanged = false;
      interrupts();
    }

    noInterrupts(); // 인터럽트 비활성화
    uart_send_string("Stepping Motor Tick: ");
    uart_send_int(stepper_motor_tick);
    uart_send_char('\n'); // 줄바꿈 문자 전송
    interrupts(); // 인터럽트 활성화
  }
}

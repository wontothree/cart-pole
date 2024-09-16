#include "uart.hpp"
#include "stpper_motor.hpp"

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
  // TCCR1B |= (1 << CS11) | (1 << CS10); // 64 분주 mode
  TCCR1B = (1 << CS11); // 8 분주

  // enable interrupts
  interrupts();

  // core logic
  float current_velocity = 1;
  uint16_t last_step_count = 0;
  uint16_t last_control_count = 0;
  uint16_t step_interval_counts = 314;
  const uint16_t MOTOR_CONTROL_COUNTS = 200;
  const uint16_t UART_UPDATE_INTERVAL = 30537; // 10초마다 출력
  uint16_t last_uart_update = 0;

  while (true) {
    // clock count
    uint16_t current_count = (uint16_t)TCNT1;

    // step motor controller
    if ((current_count - last_step_count) > step_interval_counts) {
      moveOneStep();
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

    // set direction
    if (isDirectionChanged) {
      noInterrupts();
      // delay(50);
      direction = (direction == 1) ? -1 : 1;
      isDirectionChanged = false;
      interrupts();
    }
  
    // // UART communication
    // if ((current_count - last_uart_update) > UART_UPDATE_INTERVAL) {
    //   noInterrupts();
    //   // send_uart_string("Stepping Motor Tick: ");
    //   send_uart_int(stepper_motor_tick);
    //   // send_uart_char('\n');
    //   interrupts();
    //   last_uart_update = current_count;
    // }
  }
}

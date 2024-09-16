#include "globals.hpp"
#include "stepper_motor_controller.hpp"
#include "uart.hpp"
#include "timer.hpp"

void setup() {
  initialize_uart(9600); 

  initialize_motor_pins();

  initialize_timer1(8);

  // core logic
  float current_velocity = 1;
  uint16_t last_step_count = 0;
  uint16_t last_control_count = 0;
  uint16_t step_interval_counts = 314;
  uint16_t last_uart_update = 0;

  while (true) {
    // clock count
    uint16_t current_count = get_timer1_count();

    // step motor controller
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

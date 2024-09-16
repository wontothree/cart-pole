#include "globals.hpp"
#include "stepper_motor_controller.hpp"
#include "uart.hpp"
#include "timer.hpp"

void setup() {
  initialize_uart(9600); 

  initialize_motor_pins();

  initialize_timer1(8);

  float current_velocity = 1;
  uint16_t step_interval_counts = 314;

  uint16_t last_uart_update = 0;

  while (true) {
    // clock count
    uint16_t current_count = get_timer1_count();

    update_motor_control(current_count, step_interval_counts, current_velocity, target_velocity);

    handle_motor_direction();

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

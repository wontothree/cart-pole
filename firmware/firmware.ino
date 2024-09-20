#include "globals.hpp"
#include "stepper_motor_controller.hpp"
#include "uart.hpp"
#include "timer.hpp"

void setup() {
  initialize_uart(9600); 

  initialize_motor_pins();

  initialize_timer1(PRESCALER);

  float current_velocity = 0;
  uint16_t last_uart_update = 0;

  uint16_t target_step_interval_counts = 80;
  // uint16_t target_current_linear_acceleration = 0;

  while (true) {
    // clock count
    uint16_t current_count = get_timer1_count();

    float target_velocity_copy = target_velocity;

    // update_motor_control(current_count, step_interval_counts, current_velocity, target_velocity);

    update_motor_control_by_accel(current_count, target_step_interval_counts, target_velocity_copy, target_velocity);

    handle_motor_direction();

    // UART communication
    if ((current_count - last_uart_update) > UART_UPDATE_INTERVAL) {
        noInterrupts(); // UART 송신 중 인터럽트를 비활성화
        send_uart_int(stepper_motor_tick);
        send_uart_char(', ');
        send_uart_int(stepper_motor_tick_observation_time);
        send_uart_char('\n');
        interrupts(); // 인터럽트를 다시 활성화
        last_uart_update = current_count;
    }
  }
}

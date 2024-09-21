#include "globals.hpp"
#include "stepper_motor_controller.hpp"
#include "uart.hpp"
#include "timer.hpp"

float currentPosition = 0;
float currentVelocity = 0;

float acceleration = 0;
uint16_t currentMotorInterval = 500;
uint16_t lastMotorUpdateCount = 0;

void setup() {
  initializeUart(9600);
  initializeStepperMotorPins();
  initializeTimer(64);

  // uart
  uint16_t lastUartUpdateCount = 0;
  const uint16_t UART_UPDATE_INTERVAL = 10000; 

  while (true) {
    // clock count
    uint16_t currentCount = getTimerCount();

    updateMotorByAcceleration(currentCount, acceleration);

    // UART communication
    if ((currentCount - lastUartUpdateCount) > UART_UPDATE_INTERVAL) {
        noInterrupts(); // UART 송신 중 인터럽트를 비활성화
        send_uart_int(currentPosition);
        send_uart_char(', ');
        send_uart_int(currentVelocity);
        send_uart_char('\n');
        interrupts(); // 인터럽트를 다시 활성화
        lastUartUpdateCount = currentCount;
    }
  }
}

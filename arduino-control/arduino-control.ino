#include "stepper_motor_controller.hpp"
#include "uart.hpp"
#include "timer.hpp"

float currentPosition = 0;
float currentVelocity = 0;

float acceleration = 0.1;

void setup()
{
  // initializeUart(9600);
  initializeStepperMotorPins();
  initializeTimer(64);

  // uart
  //  uint16_t lastUartUpdateCount = 0;
  // const uint16_t UART_UPDATE_INTERVAL = 10000;

  float currentVelocity = 0;
  float currentPosition = 0;

  while (true)
  {
    // clock count

    uint16_t currentCount = getTimerCount();
    updateMotorByAcceleration(currentCount, acceleration, &currentVelocity, &currentPosition);

    // // communication
    // if ((currentCount - lastUartUpdateCount) > UART_UPDATE_INTERVAL) {
    //     noInterrupts();
    //     send_uart_int(currentPosition);
    //     send_uart_char(', ');
    //     send_uart_int(currentVelocity);
    //     send_uart_char('\n');
    //     interrupts();
    //     lastUartUpdateCount = currentCount;
    // }
  }
}

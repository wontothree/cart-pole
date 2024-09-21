#include "stepper_motor_controller.hpp"
#include "uart.hpp"
#include "timer.hpp"

void setup()
{
  initializeUart(9600);
  initializeStepperMotorPins();
  initializeTimer(64);

  float acceleration = 0;
  float currentVelocity = 0;
  float currentPosition = 0;

  while (true)
  {
    uint16_t currentCount = getTimerCount();
    updateMotorByAcceleration(currentCount, acceleration, &currentVelocity, &currentPosition);
  }
}

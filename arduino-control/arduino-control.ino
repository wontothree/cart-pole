#include "stepper_motor_controller.hpp"
#include "uart.hpp"
#include "timer.hpp"
#include <stdio.h>

void setup()
{
  initializeUart(500000);
  initializeStepperMotorPins();
  initializeTimer(64);

  float acceleration = 0;
  float currentVelocity = 0;
  float currentPosition = 0;

  uint16_t loggingInterval = 1000;
  uint16_t lastLogging = 0;
  char vBuf[16];
  char pBuf[16];
  char buf[64];

  while (true)
  {
    uint16_t currentCount = getTimerCount();

    updateMotorByAcceleration(currentCount, acceleration, &currentVelocity, &currentPosition);
    if (currentCount - lastLogging > loggingInterval)
    {
      dtostrf(currentVelocity, 5, 3, vBuf);
      dtostrf(currentPosition, 5, 3, pBuf);
      sprintf(buf, "V:%s,P:%s\n", vBuf, pBuf);
      putString(buf);
      lastLogging += loggingInterval;
    }

    processUart();
  }
}

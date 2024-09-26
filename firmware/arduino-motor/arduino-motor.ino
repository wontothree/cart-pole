#include "stepper_motor_controller.hpp"
#include "uart.hpp"
#include "timer.hpp"
#include <stdio.h>

void setup()
{
  initializeUart(1000000);
  initializeStepperMotorPins();
  initializeTimer(64);

  float acceleration = 0;
  float currentVelocity = 0;
  float currentPosition = 0;

  pinMode(A0, INPUT);

  // First alignment
  while (analogRead(A0) < 100)
  {
    updateMotorByVelocity(getTimerCount(), 0.1, &currentPosition);
  }
  updateMotorByVelocity(getTimerCount(), 0, &currentPosition);
  currentPosition = 0;

  // Move to left
  while (currentPosition > -0.03f)
  {
    updateMotorByVelocity(getTimerCount(), -0.02, &currentPosition);
  }

  // Second alignment
  while (analogRead(A0) < 100)
  {
    updateMotorByVelocity(getTimerCount(), 0.01, &currentPosition);
  }
  updateMotorByVelocity(getTimerCount(), 0, &currentPosition);
  currentPosition = 0;

  // Move to home position
  currentPosition = 0.34f;
  while (currentPosition > 0)
  {
    updateMotorByVelocity(getTimerCount(), -0.05, &currentPosition);
  }
  updateMotorByVelocity(getTimerCount(), 0, &currentPosition);

  uint16_t loggingInterval = 2500;
  uint16_t lastLogging = 0;

  int sign = 1;
  int num = 0;
  uint8_t byte;

  int16_t tmp;

  while (true)
  {
    uint16_t currentCount = getTimerCount();

    // Motor control
    updateMotorByAcceleration(currentCount, acceleration, &currentVelocity, &currentPosition);

    // Command input
    if (getByte(&byte))
    {
      if (byte == '\n' || byte == '\r')
      {
        acceleration = sign * num * 0.01f;
        // sprintf(buf, "Accel:%d/100\n", sign * num);
        // putString(buf);
        sign = 1;
        num = 0;
      }
      else if (byte >= '0' && byte <= '9')
      {
        num *= 10;
        num += (byte - '0');
      }
      else if (byte == '-')
      {
        sign = -1;
      }
    }

    // Log output
    if (currentCount - lastLogging > loggingInterval)
    {
      putByte('V');
      tmp = currentVelocity * 10000;
      if (tmp < 0)
      {
        putByte('-');
        tmp *= -1;
      }
      do
      {
        putByte('0' + tmp % 10);
        tmp /= 10;
      } while (tmp);

      putByte('P');
      tmp = currentPosition * 10000;
      if (tmp < 0)
      {
        putByte('-');
        tmp *= -1;
      }
      do
      {
        putByte('0' + tmp % 10);
        tmp /= 10;
      } while (tmp);

      putByte('\n');

      lastLogging += loggingInterval;
    }

    processUart();

    // safe for test
    if (currentPosition < -0.34 || currentPosition > 0.34)
      break;
  }

  finalizeStepperMotorPins();
  while (1)
    ;
}

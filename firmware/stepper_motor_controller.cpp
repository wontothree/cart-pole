#include "stepper_motor_controller.hpp"
#include "timer.hpp"

// 1, 2 phase excitation method
int step_info[8][4] = 
{
  { HIGH, LOW, LOW, LOW }, // one phase, 0.9 degree, or 1/400 revolution
  { HIGH, HIGH, LOW, LOW },
  { LOW, HIGH, LOW, LOW },
  { LOW, HIGH, HIGH, LOW },
  { LOW, LOW, HIGH, LOW },
  { LOW, LOW, HIGH, HIGH },
  { LOW, LOW, LOW, HIGH },
  { HIGH, LOW, LOW, HIGH }
};

uint16_t lastMotorUpdateCount = getTimerCount();
float currentMotorInterval = 625;

// rotate one phase (0.9 degree)
void moveOneStep()
{
    static int step = 0;
    digitalWrite(PIN_A, step_info[step][0]);
    digitalWrite(PIN_B, step_info[step][1]);
    digitalWrite(PIN_NA, step_info[step][2]);
    digitalWrite(PIN_NB, step_info[step][3]);
    step += direction;
    step &= 0x07;
}

void initializeStepperMotorPins()
{
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_NA, OUTPUT);
    pinMode(PIN_NB, OUTPUT);
}
#define MIN_INTERVAL 30
#define TICK_PER_METER (6366.197)   // from 400 tick = 2 pi (0.01m)
#define COUNT_PER_SECOND (250000)   // 16M / 64

void updateMotorByAcceleration(uint16_t currentCount, float acceleration)
{
    // move one step 
    if (currentCount - lastMotorUpdateCount >= currentMotorInterval)
    {
        moveOneStep();

        float newMotorInterval =  currentMotorInterval * 9817478.15847f / (9817478.15847f +  acceleration * currentMotorInterval * currentMotorInterval);

        if (newMotorInterval < MIN_INTERVAL) newMotorInterval = MIN_INTERVAL;

        // Update position of cart (m) (when tick = 1)
        currentPosition += 1.0f / TICK_PER_METER;

        // Update velocity of cart (m/s)
        // (count/s) / (tick/m * count) = m/s (when tick = 1)
        currentVelocity = COUNT_PER_SECOND / (TICK_PER_METER *  currentMotorInterval);

        // update interval
        currentMotorInterval =newMotorInterval;

        // update last motor update count
        lastMotorUpdateCount += currentMotorInterval;
    }
}

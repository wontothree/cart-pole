#include "stepper_motor_controller.hpp"

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

// rotate one phase (0.9 degree)
void moveOneStep()
{
    static int step = 0;
    digitalWrite(PIN_A, step_info[step][0]);
    digitalWrite(PIN_B, step_info[step][1]);
    digitalWrite(PIN_NA, step_info[step][2]);
    digitalWrite(PIN_NB, step_info[step][3]);
    step += direction;
    if (step > 7) step = 0;
    if (step < 0) step = 7;
}

void initializeStepperMotorPins()
{
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_NA, OUTPUT);
    pinMode(PIN_NB, OUTPUT);
}

void updateMotorByAcceleration(uint16_t currentCount, float acceleration)
{
    const float TICK_PER_METER = 6366.197;   // from 400 tick = 2 pi (0.01m)
    const float COUNT_PER_SECOND = 250000;   // 16M / 64

    float newMotorInterval = (float) currentMotorInterval / (1.0f + acceleration * currentMotorInterval * TICK_PER_METER / COUNT_PER_SECOND);

    // move one step 
    if (currentCount - lastMotorUpdateCount >= currentMotorInterval)
    {
        moveOneStep();

        // Update position of cart (m) (when tick = 1)
        currentPosition += 1.0f / TICK_PER_METER;
        // Update velocity of cart (m/s)
        // (count/s) / (tick/m * count) = m/s (when tick = 1)
        currentVelocity = COUNT_PER_SECOND / (TICK_PER_METER * (float) currentMotorInterval);

        // update interval
        currentMotorInterval = (uint16_t) newMotorInterval;

        // update last motor update count
        lastMotorUpdateCount += currentMotorInterval;
    }
}

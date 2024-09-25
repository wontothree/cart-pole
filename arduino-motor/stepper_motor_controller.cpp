#include "stepper_motor_controller.hpp"
#include "timer.hpp"
#include <avr/interrupt.h> // for `noInterrupts()` and `interrupts()`
#include <Arduino.h>       // for digitalWrite and pinMode

#define MIN_INTERVAL (30)
#define TICK_PER_METER (6366.197)             // from 400 tick = 2 pi (0.01m)
#define COUNT_PER_SECOND (250000)             // 16M / 64
#define SI_ACCEL_TO_HW_ACCEL (9817478.15847f) // m/s^2 을 step/tick^2으로 변환한다. 구체적으로는 COUNT_PER_SECOND^2/TICK_PER_METER 이다.

#define PIN_A 10
#define PIN_NA 11
#define PIN_B 12
#define PIN_NB 13

// 1, 2 phase excitation method
int step_info[8][4] = {{HIGH, LOW, LOW, LOW}, // one phase, 0.9 degree, or 1/400 revolution
                       {HIGH, HIGH, LOW, LOW},
                       {LOW, HIGH, LOW, LOW},
                       {LOW, HIGH, HIGH, LOW},
                       {LOW, LOW, HIGH, LOW},
                       {LOW, LOW, HIGH, HIGH},
                       {LOW, LOW, LOW, HIGH},
                       {HIGH, LOW, LOW, HIGH}};

static int direction = 1;
static uint16_t lastMotorUpdateCount = getTimerCount();
static float currentMotorInterval = 625; // 초당 1바퀴를 돌도록 설정

// rotate one phase (0.9 degree)
static inline void moveOneStep()
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

void updateMotorByAcceleration(uint16_t currentCount, float acceleration, float *currentVelocity, float *currentPosition)
{
    // move one step
    if (currentCount - lastMotorUpdateCount >= currentMotorInterval)
    {
        moveOneStep();

        float newMotorInterval = currentMotorInterval * SI_ACCEL_TO_HW_ACCEL / (SI_ACCEL_TO_HW_ACCEL + acceleration * currentMotorInterval * currentMotorInterval);

        if (newMotorInterval < MIN_INTERVAL)
            newMotorInterval = MIN_INTERVAL;

        // update position of cart (m) (when tick = 1)
        *currentPosition += 1.0f / TICK_PER_METER;

        // update velocity of cart (m/s)
        // (count/s) / (tick/m * count) = m/s (when tick = 1)
        *currentVelocity = COUNT_PER_SECOND / (TICK_PER_METER * currentMotorInterval);

        // update interval
        currentMotorInterval = newMotorInterval;

        // update last motor update count
        lastMotorUpdateCount += currentMotorInterval;
    }
}

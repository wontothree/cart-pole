#include "stepper_motor_controller.hpp"
#include "timer.hpp"
#include <avr/interrupt.h> // for `noInterrupts()` and `interrupts()`
#include <Arduino.h>       // For digitalWrite and pinMode

#define MIN_INTERVAL (30)
#define METER_PER_STEP (6366.198)       // from 400 step = 2 pi (0.01 m), 1 m = 6366.198 step
#define COUNT_PER_SECOND (250000)       // 16M / 64, second = count x 1/(16M/64)
#define SI_ACCEL_TO_HW_ACCEL (10937500) // SI unit for all unit - step, count

#define PIN_A 10
#define PIN_NA 11
#define PIN_B 12
#define PIN_NB 13

#define sign(x) (((x) > 0) ? 1 : -1)

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
static float currentMotorInterval = 65537; // 초당 1바퀴를 돌도록 설정

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
    if (abs(currentMotorInterval) < 10000)
    {
        if (currentCount - lastMotorUpdateCount >= currentMotorInterval)
        {
            moveOneStep();

            float nextMotorInterval = currentMotorInterval * SI_ACCEL_TO_HW_ACCEL / (SI_ACCEL_TO_HW_ACCEL + direction * acceleration * currentMotorInterval * currentMotorInterval);

            // if (nextMotorInterval < MIN_INTERVAL)
            //     nextMotorInterval = MIN_INTERVAL;

            // update velocity of cart (m/s)
            // (count/s) / (tick/m * count) = m/s (when step = 1)
            *currentVelocity = direction * COUNT_PER_SECOND / (METER_PER_STEP * currentMotorInterval);

            // update position of cart (m) (when step = 1)
            *currentPosition += 1.0f / METER_PER_STEP;

            // update interval
            currentMotorInterval = nextMotorInterval;

            // update last motor update count
            lastMotorUpdateCount += currentMotorInterval;
        }
    }
    else
    {
        if (currentCount - lastMotorUpdateCount >= 100)
        {
            *currentVelocity += acceleration * 100 / COUNT_PER_SECOND;
            direction = sign(*currentVelocity);
            currentMotorInterval = direction * COUNT_PER_SECOND / (METER_PER_STEP * *currentVelocity);
        }
    }
}

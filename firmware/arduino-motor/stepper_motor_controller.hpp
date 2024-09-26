#pragma once

#include <stdint.h>

// pin map
void initializeStepperMotorPins();
void finalizeStepperMotorPins();

/*
 * 모터를 가속도에 따라 제어하고, 현재 속도와 틱을 업데이트한다.
 * @param currentCounter 타이머로부터 공급되는 카운터
 * @param acceleration 가속도 (단위 m/s^2)
 * @param currentVelocity 현재 속도 (output)
 * @param currentPosition 현재 위치 (output)
 *
 * 주의: 현재 위치는 내부적으로 tracking하지 않으며, 외부에서 공급된 값에 변화만을 가한다.
 */
void updateMotorByAcceleration(uint16_t currentCounter, float acceleration, float *currentVelocity, float *currentPosition);
void updateMotorByVelocity(uint16_t currentCount, float velocity, float *currentPosition);

#include "pid.h"

float targetAngle;
float angleError;
float kp, ki, kd;

float cumulativeAngleError;
float lastAngleError;

float lastTime = 0;

void setPID(float KP, float KI, float KD)
{
    kp = KP;
    ki = KI;
    kd = KD;
}

void setTargetAngle(float targetAngle)
{
    targetAngle = targetAngle;
}

float controlByPID(float angle)
{
    // proportional term
    angleError = angle - targetAngle;

    float currentTime = millis();

    float timeInterval = currentTime - lastTime;

    // integral term
    cumulativeAngleError += angleError * timeInterval;

    // derivative term
    float derivativeAngleError = (angleError - lastAngleError) / timeInterval;

    float output = kp * angleError + ki * cumulativeAngleError + kd * derivativeAngleError;

    // update last values
    lastAngleError = angleError;
    lastTime = currentTime;

    return output;
}

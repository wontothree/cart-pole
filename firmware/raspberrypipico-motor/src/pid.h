#ifndef PID_H
#define PID_H

// set constant parameter gain of PID controller
void setPID(float KP, float KI, float KD);

// set target angle of pole
void setTargetAngle(float TargetAngle);

float controlByPID(float angle);

#endif
# Research Note

2024.09.25

I shoud start with balancing using only the Arduino without any external communication. I shoud not try to do too much at once. I need to take it step by step.

I realized I can never apply nonlinear model predictive control algorithm to my completed cart pole system at once although i have achieved the algorithm in simulation environment. I decide to test step by step from most simple balancing algorithm in my hardware.

P controller using only angle of pole obtained from absolute rotary encoder. sudo code is like

```cpp
if (angle > 0 ) { // clockwise is positive
    // move right as constant velocity
} else (angle < 0) {
    // move left as constant velocity
}
```

PID controller giving velocity of cart

```cpp
// parametric gain constant
float angleError;
float kp, ki, kd;
float cumulativeAngleError;
float lastAngleError;

// set parametric constant of pid controller
void setPID(float KP, float KI, float KD) 
{
    // proportional constant
    kp = KP;

    // integral constant
    ki = KI;

    // derivative constant
    kd = KD;
}

float controlByPID()
{
    // proportional
    angleError = abs(angle - targetAngle);

    float currentTime = millis();

    float timeInterval = currentTime - lastTime;

    // integral
    float cumulativeAngleError = angleError * timeInterval;

    // derivative
    float derivativeAngleError = (angleError - lastAngleError) / timeInterval;

    // output of pid
    float output = kp * angleError + ki * cumulativeAngleError + kd * derivativeAngleError;

    // update last values
    lastAngleError = angleError;
    lastTime = currentTime;

    return output
}

void loop()
{
    float interval = constant * controlByPID();

    // limit max and min
    if (interval < constant) {
        interval = constant;
    } else if (interval > constant) {
        interval = constant;
    }

    if (angleError > 0) {
        // move right
    } else if (angleError < 0) {
        // move left
    }
}
```

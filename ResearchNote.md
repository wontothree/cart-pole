<<<<<<< HEAD
=======
<<<<<<< HEAD
# Research Note

*2024.09.25*

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
float targetAngle;

// parametric gain constant
float angleError;
float kp, ki, kd;

float cumulativeAngleError;
float lastAngleError;

// set parametric constant of pid controller
void setPID(float KP, float KI, float KD) 
{
    kp = KP;    // proportional constant
    ki = KI;    // integral constant
    kd = KD;    // derivative constant
}

void setTargetAngle(targetAngle) {
    targetAngle = targetAngle
}

float controlByPID()
{
    // proportional term
    angleError = angle - targetAngle;

    float currentTime = millis();

    float timeInterval = currentTime - lastTime;

    // integral term
    cumulativeAngleError += angleError * timeInterval;

    // derivative term
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
    float acceleration = controlByPID(); 

    float maxAcceleration = 10.0; 
    float minAcceleration = -10.0;

    // limit max and min
    if (interval > maxAcceleration) {
        interval = maxAcceleration;
    } else if (interval < minAcceleration) {
        interval = minAcceleration;
    }

    // motor control

    // 
=======
>>>>>>> 15c2c0614a5f5b6d5af3bee719407c1dd0325904
# Research Note

_2024.09.25_

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
float targetAngle;

// parametric gain constant
float angleError;
float kp, ki, kd;

float cumulativeAngleError;
float lastAngleError;

// set parametric constant of pid controller
void setPID(float KP, float KI, float KD)
{
    kp = KP;    // proportional constant
    ki = KI;    // integral constant
    kd = KD;    // derivative constant
}

void setTargetAngle(targetAngle) {
    targetAngle = targetAngle
}

float controlByPID()
{
    // proportional term
    angleError = angle - targetAngle;

    float currentTime = millis();

    float timeInterval = currentTime - lastTime;

    // integral term
    cumulativeAngleError += angleError * timeInterval;

    // derivative term
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
    float acceleration = controlByPID();

    float maxAcceleration = 10.0;
    float minAcceleration = -10.0;

    // limit max and min
    if (interval > maxAcceleration) {
        interval = maxAcceleration;
    } else if (interval < minAcceleration) {
        interval = minAcceleration;
    }

    // motor control

    // if (angleError > 0) {
    //     // move right
    // } else if (angleError < 0) {
    //     // move left
    // }
}
```

What shoud this pid controller return? velocity or acceleration? I think that is acceleration. Tho it's suspicious.

지글로 니콜스

<<<<<<< HEAD
_2024.09.26_

raw encoder data : counter clockwise = (+) direction in angle of pole
=======
*2024.09.26*

I make timing belt tight.

USB data(position, velcity) serial data and uart data (encoder data) are miexed.
>>>>>>> 15c2c0614a5f5b6d5af3bee719407c1dd0325904

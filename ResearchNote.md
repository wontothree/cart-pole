# Research Note

2024.09.25

I shoud start with balancing using only the Arduino without any external communication. I shoud not try to do too much at once. I need to take it step by step.

I realized I can never apply nonlinear model predictive control algorithm to my completed cart pole system although i have achieved the algorithm in simulation environment. I decide to test step by step from most simple balancing algorithm in my hardware.

p controller using only angle of pole obtained from absolute rotary encoder. sudo code is like

```cpp
if (angle > 0 ) { // clockwise is positive
    // move right as constant velocity
} else (angle <0) {
    // move left as constant velocity
}
```

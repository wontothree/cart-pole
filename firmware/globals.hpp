#ifndef GLOBALS_HPP
#define GLOBALS_HPP

// define pin numbers
#define PIN_A 10
#define PIN_NA 11
#define PIN_B 12
#define PIN_NB 13

// global variables
extern volatile int direction;        // 1 : clockwise, -1 : counter-clockwise
extern volatile bool isDirectionChanged;
extern volatile float target_velocity;

#endif // GLOBALS_HPP

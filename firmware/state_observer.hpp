#ifdef STATE_OBSERVER_HPP
#define STATE_OBSERVER_HPP

#include "globals.hpp"

void calculate_cart_position_and_velocity(int16_t stepper_motor_tick, unsigned long stepper_motor_tick_observation_time);

#endif // STATE_OBSERVER_HPP
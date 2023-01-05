#ifndef GUARD_motors_h
#define GUARD_motors_h

#include "debug.h"

// Sets pinMode for motor control pins to OUTPUT
void motors_init_pins();

// Increment (or decrement if argument is negative) the speed of the motors.
void motors_increment_velocity_left(const char increment);
void motors_increment_velocity_right(const char increment);

// The following 4 functions can be used to control the velocity of one of the wheels.
// They accept an integer in the range [-100, 100]. Negative values are used to
// set the motor in reverse. An integer outside of the range [-100, 100] will be
// changed to 100 or -100, whichever is closer.
// 
// These function update the values of current_velocity_[left or right]_[front or back]
//
// Low voltages can produce no movement, so the actual voltages applied to
// the motors are abstracted away. The intent is that a velocity of 1 should
// correspond to the minimum voltage necessary to get the robot moving.
//
// EXAMPLES
// ========
// Halting the front left wheel:
//     motors_set_velocity_left_front(0);
//
// Turning Right at 80% of max speed:
//     motors_set_velocity_left(80);
//     motors_set_velocity_right(-80); 
void motors_set_velocity_left_front(const char velocity);
void motors_set_velocity_left_back(const char velocity);
void motors_set_velocity_right_front(const char velocity);
void motors_set_velocity_right_back(const char velocity);

// Convenience function to set the velocity of all motors at once.
// May be unnecessary, could be removed if unused
void motors_set_velocity(const char velocity);

// Convenience function to set the velocity of both left motors.
void motors_set_velocity_left(const char velocity);

// Convenience function to set the velocity of both right motors simultaneously.
void motors_set_velocity_right(const char velocity);

#endif

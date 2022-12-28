#ifndef GUARD_motors_h
#define GUARD_motors_h

#include <Arduino.h>

// Sets pinMode for motor control pins to OUTPUT
void motors_init_pins();

// The following functions can be used to control the velocity of one of the wheels.
// They accept an integer in the range [-100, 100]. Negative values are used to
// set the motor in reverse.
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
// May be unnecessary, could be removed if unused
void motors_set_velocity_left(const char velocity);

// Convenience function to set the velocity of both right motors simultaneously.
// May be unnecessary, could be removed if unused
void motors_set_velocity_right(const char velocity);

#endif

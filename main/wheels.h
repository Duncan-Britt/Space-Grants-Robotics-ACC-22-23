#ifndef GUARD_wheels_h
#define GUARD_wheels_h

#include <Arduino.h>

class Wheels {
public:
  // Sets motor control pins for the wheels to OUTPUT, initializes members which refer to wheel motor control pins. 
  Wheels(const byte left_forward, const byte left_backward, const byte right_forward, const byte right_backward, const byte motor_speed, const byte motor_speed2) : 
         pin_left_fw(left_forward), pin_left_bw(left_backward), pin_right_fw(right_forward), pin_right_bw(right_backward), pin_motor_speed(motor_speed), pin_motor_speed2(motor_speed2) {
    pinMode(left_forward, OUTPUT);
    pinMode(left_backward, OUTPUT);
    pinMode(right_forward, OUTPUT);
    pinMode(right_backward, OUTPUT);
    pinMode(motor_speed, OUTPUT);
    pinMode(motor_speed2, OUTPUT);
  }
  void go();
  void stop(); // Need to investigate breaking. Currently just stops voltage to the wheels.
  void reverse();
  void turn_right(); // Tank turns:
  void turn_left();  // One side spins forward, the other backward.
  void set_speed(byte new_speed);
  static const byte FAST = 255;   // Example use: 
  static const byte MEDIUM = 180; // wheels.set_speed(Wheels::MEDIUM); 
  static const byte SLOW = 120;   //
private:
  byte speed = FAST; // Default speed
  byte pin_left_fw;
  byte pin_left_bw;
  byte pin_right_fw;
  byte pin_right_bw;
  byte pin_motor_speed;
  byte pin_motor_speed2;
};

#endif
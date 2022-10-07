#include "wheels.h"

void Wheels::go() {
  Serial.println("Going!");
  digitalWrite(pin_left_bw, LOW);
  digitalWrite(pin_right_bw, LOW);
  digitalWrite(pin_left_fw, HIGH);
  digitalWrite(pin_right_fw, HIGH);
  analogWrite(pin_motor_speed, speed);
  analogWrite(pin_motor_speed2, speed);
}

void Wheels::stop() {
  Serial.println("Stopped.");
  digitalWrite(pin_left_bw, LOW);
  digitalWrite(pin_right_bw, LOW);
  digitalWrite(pin_left_fw, LOW);
  digitalWrite(pin_right_fw, LOW);
  analogWrite(pin_motor_speed, 0);
  analogWrite(pin_motor_speed2, 0);
}

void Wheels::reverse() {
  Serial.println("Backing up.");
  digitalWrite(pin_left_fw, LOW);
  digitalWrite(pin_right_fw, LOW);
  digitalWrite(pin_left_bw, HIGH);
  digitalWrite(pin_right_bw, HIGH);
  analogWrite(pin_motor_speed, speed);
  analogWrite(pin_motor_speed2, 0);
}

void Wheels::turn_right() {
  Serial.println("Right Turn");
  stop();
  digitalWrite(pin_right_bw, HIGH);
  digitalWrite(pin_left_fw, HIGH);
  analogWrite(pin_motor_speed, SLOW);
  analogWrite(pin_motor_speed2, SLOW);
}

void Wheels::turn_left() {
  Serial.println("Left Turn");
  stop();
  digitalWrite(pin_left_bw, HIGH);
  digitalWrite(pin_right_fw, HIGH);
  analogWrite(pin_motor_speed, SLOW);
  analogWrite(pin_motor_speed2, SLOW);
}

void Wheels::set_speed(byte new_speed) {
  if (speed < 0 || speed > 255) { // Invalid speed
    return;
  }

  if (speed == 0) { // If not currently moving.
    speed = new_speed;
    return;
  }

  analogWrite(pin_motor_speed, speed);
  speed = new_speed;
}
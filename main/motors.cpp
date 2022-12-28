#include "motors.h"
#include <Arduino.h>

// The following constants are TBD
static const byte pin_left_front_fw = 8;
static const byte pin_left_front_bw = 9;
static const byte pin_left_back_fw = 10;
static const byte pin_left_back_bw = 11;
static const byte pin_right_front_fw = 12;
static const byte pin_right_front_bw = 13;
static const byte pin_right_back_fw = 14;
static const byte pin_right_back_bw = 15;
static const byte pin_left_front_speed = 16;
static const byte pin_left_back_speed = 17;
static const byte pin_right_front_speed = 18;
static const byte pin_right_back_speed = 19;

void motors_init_pins()
{
    pinMode(pin_left_front_fw, OUTPUT);
    pinMode(pin_left_front_bw, OUTPUT);
    pinMode(pin_left_back_fw, OUTPUT);
    pinMode(pin_left_back_bw, OUTPUT);
    pinMode(pin_right_front_fw, OUTPUT);
    pinMode(pin_right_front_bw, OUTPUT);
    pinMode(pin_right_back_fw, OUTPUT);
    pinMode(pin_right_back_bw, OUTPUT);
    pinMode(pin_left_front_speed, OUTPUT);
    pinMode(pin_left_back_speed, OUTPUT);
    pinMode(pin_right_front_speed, OUTPUT);
    pinMode(pin_right_back_speed, OUTPUT);
}

// Maps values in range [0, 100] to [min_movement, max_voltage]. Internal to this file.
byte voltage_of_speed(const byte speed)
{
    const byte min_movement_voltage = 30; // TBD experimentally
    const byte max_voltage = 255; // Max Analogue Pin Output
    
    return (speed * (max_voltage - min_movement_voltage)) / 100 + min_movement_voltage;
}

void motors_set_velocity_left_front(const char velocity)
{    
    if (velocity == 0) {
        digitalWrite(pin_left_front_fw, LOW);
        digitalWrite(pin_left_front_bw, LOW);
        analogWrite(pin_left_front_speed, 0);
    }
    else if (velocity > 0) {
        digitalWrite(pin_left_front_fw, HIGH);
        digitalWrite(pin_left_front_bw, LOW);
        analogWrite(pin_left_front_speed, voltage_of_speed((byte)abs(velocity)));
    } else {
        digitalWrite(pin_left_front_fw, LOW);
        digitalWrite(pin_left_front_bw, HIGH);
        analogWrite(pin_left_front_speed, voltage_of_speed((byte)abs(velocity)));
    }
}

void motors_set_velocity_left_back(const char velocity)
{    
    if (velocity == 0) {
        digitalWrite(pin_left_back_fw, LOW);
        digitalWrite(pin_left_back_bw, LOW);
        analogWrite(pin_left_back_speed, 0);
    }
    else if (velocity > 0) {
        digitalWrite(pin_left_back_fw, HIGH);
        digitalWrite(pin_left_back_bw, LOW);
        analogWrite(pin_left_back_speed, voltage_of_speed((byte)abs(velocity)));
    } else {
        digitalWrite(pin_left_back_fw, LOW);
        digitalWrite(pin_left_back_bw, HIGH);
        analogWrite(pin_left_back_speed, voltage_of_speed((byte)abs(velocity)));
    }
}

void motors_set_velocity_right_front(const char velocity)
{    
    if (velocity == 0) {
        digitalWrite(pin_right_front_fw, LOW);
        digitalWrite(pin_right_front_bw, LOW);
        analogWrite(pin_right_front_speed, 0);
    }
    else if (velocity > 0) {
        digitalWrite(pin_right_front_fw, HIGH);
        digitalWrite(pin_right_front_bw, LOW);
        analogWrite(pin_right_front_speed, voltage_of_speed((byte)abs(velocity)));
    } else {
        digitalWrite(pin_right_front_fw, LOW);
        digitalWrite(pin_right_front_bw, HIGH);
        analogWrite(pin_right_front_speed, voltage_of_speed((byte)abs(velocity)));
    }
}

void motors_set_velocity_right_back(const char velocity)
{    
    if (velocity == 0) {
        digitalWrite(pin_right_back_fw, LOW);
        digitalWrite(pin_right_back_bw, LOW);
        analogWrite(pin_right_back_speed, 0);
    }
    else if (velocity > 0) {
        digitalWrite(pin_right_back_fw, HIGH);
        digitalWrite(pin_right_back_bw, LOW);
        analogWrite(pin_right_back_speed, voltage_of_speed((byte)abs(velocity)));
    } else {
        digitalWrite(pin_right_back_fw, LOW);
        digitalWrite(pin_right_back_bw, HIGH);
        analogWrite(pin_right_back_speed, voltage_of_speed((byte)abs(velocity)));
    }
}

void motors_set_velocity_left(const char velocity)
{
    motors_set_velocity_left_front(velocity);
    motors_set_velocity_left_back(velocity);
}

void motors_set_velocity_right(const char velocity)
{
    motors_set_velocity_right_front(velocity);
    motors_set_velocity_right_back(velocity);
}

void motors_set_velocity(const char velocity)
{
    motors_set_velocity_left(velocity);
    motors_set_velocity_right(velocity);
}

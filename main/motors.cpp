#include "motors.h"
#include <Arduino.h>

// ___     ___ _  _               
//  |  |\ | | |_ |_) |\ |  /\  |  
// _|_ | \| | |_ | \ | \| /--\ |_

// PINS: The following constants are TBD
// Yet TBD if the motors on either side may share any pins
// The following code assumes they won't.
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

static char current_velocity_left_front = 0;
static char current_velocity_left_back = 0;
static char current_velocity_right_front = 0;
static char current_velocity_right_back = 0;

// Maps values in range [0, 100] to [min_movement, max_voltage].
byte voltage_of_speed(const byte speed)
{
    const byte min_movement_voltage = 30; // TBD experimentally
    const byte max_voltage = 255; // Max Analogue Pin Output
    
    return (speed * (max_voltage - min_movement_voltage)) / 100 + min_movement_voltage;
}

// Return velocity bounded within range [-100, 100].
char normalize(const char velocity)
{
    return velocity > 100 ? 100 : (velocity < -100 ? -100 : velocity);
}

//  _   _  _ ___      ___ ___ ___  _        __   
// | \ |_ |_  |  |\ |  |   |   |  / \ |\ | (_  o 
// |_/ |_ |  _|_ | \| _|_  |  _|_ \_/ | \| __) o
//==============================================
//  _       _     ___  _         _ ___ 
// |_) | | |_) |   |  /     /\  |_) |  
// |   |_| |_) |_ _|_ \_   /--\ |  _|_ 

// Sets pinMode for motor control pins to OUTPUT
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

// Increment (or decrement if argument is negative) the speed of the motors.
void motors_increment_velocity_left(const char increment)
{
    motors_set_velocity_left_front(current_velocity_left_front + increment);
    motors_set_velocity_left_back(current_velocity_left_back + increment);
}

void motors_increment_velocity_right(const char increment)
{
    motors_set_velocity_right_front(current_velocity_right_front + increment);
    motors_set_velocity_right_back(current_velocity_right_back + increment);
}

void motors_set_velocity_left_front(char velocity)
{
    velocity = normalize(velocity);
    
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

    current_velocity_left_front = velocity;
}

void motors_set_velocity_left_back(char velocity)
{
    velocity = normalize(velocity);
        
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

    current_velocity_left_back = velocity;
}

void motors_set_velocity_right_front(char velocity)
{
    velocity = normalize(velocity);
    
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

    current_velocity_right_front = velocity;
}

void motors_set_velocity_right_back(char velocity)
{
    velocity = normalize(velocity);
    
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

    current_velocity_right_back = velocity;
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

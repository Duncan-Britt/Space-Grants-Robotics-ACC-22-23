#include "motors.h"

// ___     ___ _  _               
//  |  |\ | | |_ |_) |\ |  /\  |  
// _|_ | \| | |_ | \ | \| /--\ |_

static const byte pin_left_fw = 8; // in1
static const byte pin_left_bw = 7; // in2 
static const byte pin_left_speed = 10; // enA

static const byte pin_right_fw = 3; // in4
static const byte pin_right_bw = 5; // in3
static const byte pin_right_speed = 11; // enB

static int current_velocity_left = 0;
static int current_velocity_right = 0;

// Maps values in range [0, 100] to [min_movement, max_voltage].
int voltage_of_speed(const int speed)
{
    const int min_movement_voltage = 200; // TBD experimentally
    const int max_voltage = 255; // Max Analogue Pin Output
    
    return (speed * (max_voltage - min_movement_voltage)) / 100 + min_movement_voltage;
}

// Return velocity bounded within range [-100, 100].
int normalize(const int velocity)
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
    pinMode(pin_left_fw, OUTPUT);
    pinMode(pin_left_bw, OUTPUT);
    pinMode(pin_right_fw, OUTPUT);
    pinMode(pin_right_bw, OUTPUT);
    pinMode(pin_left_speed, OUTPUT);
    pinMode(pin_right_speed, OUTPUT);
}

// Increment (or decrement if argument is negative) the speed of the motors.
void motors_increment_velocity_left(const int increment)
{
    motors_set_velocity_left(current_velocity_left + increment);
}

void motors_increment_velocity_right(const int increment)
{
    motors_set_velocity_right(current_velocity_right + increment);
}       

void motors_set_velocity_right(int velocity)
{
    velocity = normalize(velocity);
    DEBUG_PRINTLN("right_speed: " + String(velocity));
    
    if ((int)velocity == 0) {
        digitalWrite(pin_right_fw, LOW);
        digitalWrite(pin_right_bw, LOW);
        analogWrite(pin_right_speed, 0);
    }
    else if ((int)velocity > 0) {
        digitalWrite(pin_right_fw, HIGH);
        digitalWrite(pin_right_bw, LOW);
        analogWrite(pin_right_speed, voltage_of_speed(abs(velocity)));
    } else {
        DEBUG_PRINTLN("I'm here!");
        digitalWrite(pin_right_fw, LOW);
        digitalWrite(pin_right_bw, HIGH);
        analogWrite(pin_right_speed, voltage_of_speed(abs(velocity)));
    }

    current_velocity_right = velocity;
}

void motors_set_velocity_left(int velocity)
{
    velocity = normalize(velocity);
    DEBUG_PRINTLN("left_speed: " + String(velocity));
    
    if (velocity == 0) {
        digitalWrite(pin_left_fw, LOW);
        digitalWrite(pin_left_bw, LOW);
        analogWrite(pin_left_speed, 0);
    }
    else if (velocity > 0) {
        digitalWrite(pin_left_fw, HIGH);
        digitalWrite(pin_left_bw, LOW);
        analogWrite(pin_left_speed, voltage_of_speed(abs(velocity)));
    } else {
        digitalWrite(pin_left_fw, LOW);
        digitalWrite(pin_left_bw, HIGH);
        analogWrite(pin_left_speed, voltage_of_speed(abs(velocity)));
    }

    current_velocity_left = velocity;
}

void motors_set_velocity(const int velocity)
{
    motors_set_velocity_left(velocity);
    motors_set_velocity_right(velocity);
}

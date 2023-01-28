#include "motors.h"

// ___     ___ _  _               
//  |  |\ | | |_ |_) |\ |  /\  |  
// _|_ | \| | |_ | \ | \| /--\ |_

static const byte pin_left_fw = 8; // in3 or in1?
static const byte pin_left_bw = 7; // in4 or in2?
static const byte pin_left_speed = 9; // enB or enA?

static const byte pin_right_fw = 4; // in3 or in1?
static const byte pin_right_bw = 5; // in4 or in2?
static const byte pin_right_speed = 3; // enB or enA?

static char current_velocity_left = 0;
static char current_velocity_right = 0;

// Maps values in range [0, 100] to [min_movement, max_voltage].
byte voltage_of_speed(const byte speed)
{
    const byte min_movement_voltage = 70; // TBD experimentally
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
    pinMode(pin_left_fw, OUTPUT);
    pinMode(pin_left_bw, OUTPUT);
    pinMode(pin_right_fw, OUTPUT);
    pinMode(pin_right_bw, OUTPUT);
    pinMode(pin_left_speed, OUTPUT);
    pinMode(pin_right_speed, OUTPUT);
}

// Increment (or decrement if argument is negative) the speed of the motors.
void motors_increment_velocity_left(const char increment)
{
    motors_set_velocity_left(current_velocity_left + increment);
}

void motors_increment_velocity_right(const char increment)
{
    motors_set_velocity_right(current_velocity_right + increment);
}       

void motors_set_velocity_right(char velocity)
{
    velocity = normalize(velocity);
    
    if (velocity == 0) {
        digitalWrite(pin_right_fw, LOW);
        digitalWrite(pin_right_bw, LOW);
        analogWrite(pin_right_speed, 0);
    }
    else if (velocity > 0) {
        digitalWrite(pin_right_fw, HIGH);
        digitalWrite(pin_right_bw, LOW);
        analogWrite(pin_right_speed, voltage_of_speed((byte)abs(velocity)));
    } else {
        digitalWrite(pin_right_fw, LOW);
        digitalWrite(pin_right_bw, HIGH);
        analogWrite(pin_right_speed, voltage_of_speed((byte)abs(velocity)));
    }

    current_velocity_right = velocity;
}

void motors_set_velocity_left(char velocity)
{
    velocity = normalize(velocity);
    
    if (velocity == 0) {
        digitalWrite(pin_left_fw, LOW);
        digitalWrite(pin_left_bw, LOW);
        analogWrite(pin_left_speed, 0);
    }
    else if (velocity > 0) {
        digitalWrite(pin_left_fw, HIGH);
        digitalWrite(pin_left_bw, LOW);
        analogWrite(pin_left_speed, voltage_of_speed((byte)abs(velocity)));
    } else {
        digitalWrite(pin_left_fw, LOW);
        digitalWrite(pin_left_bw, HIGH);
        analogWrite(pin_left_speed, voltage_of_speed((byte)abs(velocity)));
    }

    current_velocity_left = velocity;
}

void motors_set_velocity(const char velocity)
{
    motors_set_velocity_left(velocity);
    motors_set_velocity_right(velocity);
}

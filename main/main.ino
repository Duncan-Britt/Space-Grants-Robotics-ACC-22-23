#include "SharpIR.h"
#include "motors.h"
#include "async.h"
#include "debug.h"
#include "utility.h"

#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "imumaths.h"

#include "SimpleKalmanFilter.h"

#ifdef DEBUG
/* #include "test.h" // Used for testing */
#endif//DEBUG

//  ___       _        
//   |  _    | \  _  o 
//   | (_)   |_/ (_) o 

/* speed up and slow down loops */
/* set up the 9DOF */
/* Add calibration code */

//   _           ___       _        
//  |_ ._   _|    |  _    | \  _    
//  |_ | | (_|    | (_)   |_/ (_) o

//   _        _  _      ___  _        
//  / \ \  / |_ |_) \  / |  |_ \    / 
//  \_/  \/  |_ | \  \/ _|_ |_  \/\/  
// 


// MODULES: (Updated Jan. 1)
// _______________________________________________________________________________________
// async.h: 
// ========
// There is no multi-threading/multi-process support on the Arduino, so this code implementes an interface 
// for defining distinct loops that will run asynchronously and independently of one another.

// motors.h & motors.cpp
// =====================
// Defines an interface for incrementing and decrementing the velocity of the wheels.

// test.h
// ======
// A place to store code for testing

// debug.h
// =======
// macros for test/debug only actions.

// MISC NOTES:
// _______________________________________________________________________________________
// The code in the setup() function is run when the robot is turned on.
// Once the execution is finished, the loop() function runs until the robot is shut off.

//   _           _                         
//  |_ ._   _|   / \    _  ._   o  _       
//  |_ | | (_|   \_/ \/ (/_ | \/ | (/_ \/\/

SimpleKalmanFilter kalman_filter_heading(2, 2, 0.01);
SimpleKalmanFilter kalman_filter_acceleration(2, 2, 0.01);

// ------
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

SharpIR sensor_left = SharpIR(A0, 1080);
SharpIR sensor_right = SharpIR(A1, 1080);
SharpIR sensor_left_chassis = SharpIR(A2, 1080);
SharpIR sensor_right_chassis = SharpIR(A3, 1080);

bool time_elapsed_ms_10() {
    static uint32_t prev_millis = millis();

    if (millis() - prev_millis >= 10) {
         prev_millis = millis();
        return true;
    }

    return false; 
}

bool time_elapsed_ms_50() {
    static uint32_t prev_millis = millis();

    if (millis() - prev_millis >= 50) {
         prev_millis = millis();
        return true;
    }

    return false; 
}

bool time_elapsed_ms_70() {
    static uint32_t prev_millis = millis();

    if (millis() - prev_millis >= 50) {
         prev_millis = millis();
        return true;
    }

    return false; 
}

/* uint8_t rn = 0; // most recent distance reading for downward (hole) sensor */
/* uint8_t rm = 0; // second most recent distance reading for downward (hole) sensor */
/* uint8_t rl = 0; // third most recent distance reading for downard (hole) sensor */
/* const uint8_t r_threshold = 100; // WHAT SHOULD THIS BE? WE DON'T KNOW YET */
/* const uint8_t dr_threshold = 15; // WHAT SHOULD THIS BE? WE DON'T KNOW YET */
/* const uint8_t r_expected = 80; // WHAT SHOULD THIS BE? WE DON'T KNOW YET */

/* bool path_interrupted_by_hole() */
/* { */
/*     return rn > r_threshold && */
/*         (rn - rm > dr_threshold && */
/*          rn - rl > dr_threshold); */
/* } */

bool time_elapsed_ms_1000()
{
    static uint32_t prev_millis = millis();

    if (millis() - prev_millis >= 1000) {
         prev_millis = millis();
        return true;
    }

    return false;
}

bool time_elapsed_ms_2000()
{
    static uint32_t prev_millis = millis();

    if (millis() - prev_millis >= 2000) {
         prev_millis = millis();
        return true;
    }

    return false;
}

bool path_interrupted()    
{
    /* return false; */
    return sensor_left.distance() < 5 || sensor_right.distance() < 5;
}

AsyncLoop loop_pid_heading;
AsyncLoop loop_turn_right;
AsyncLoop loop_turn_left;
AsyncLoop loop_heading;

double heading_current;
double heading_desired;
double heading_original;

double heading_discrepancy = 0;
double heading_discrepancy_delta = 0;
double heading_discrepancy_sum = 0;

unsigned long initial_time = 0;
void* path_interruption_sequence[8];
uint8_t path_interruption_sequence_idx = 0;
void* path_interruption_sequence_2[8];
int path_interruption_sequence_idx_2 = 0;

bool bno_connected = false;

void setup() 
{
    DEBUG_BEGIN(9600); // Needed to print to Serial Monitor.
    Serial.println("here");
    /* Initialise the sensor */
    if(!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      DEBUG_PRINTLN("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    else {
      bno.setExtCrystalUse(true);
      bno_connected = true;

      delay(5000);
      sensors_event_t event_magnet;
      bno.getEvent(&event_magnet, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      float heading_measured = 180 * atan2(event_magnet.orientation.y, event_magnet.orientation.x) / PI;
      heading_current = kalman_filter_heading.updateEstimateAngle(heading_measured);
      heading_desired = heading_current;
      heading_original = heading_desired;
    }  

    loop_pid_heading
        .when((void*)time_elapsed_ms_50)
        .then((void*)+[]() -> void {
                if (bno_connected) {
                    // ask how much time actually passed, independant of when clause
                    sensors_event_t event_magnet;
                    bno.getEvent(&event_magnet, Adafruit_BNO055::VECTOR_MAGNETOMETER);
                    float heading_measured = 180 * atan2(event_magnet.orientation.y, event_magnet.orientation.x) / PI;
                    heading_current = kalman_filter_heading.updateEstimateAngle(heading_measured);
                    DEBUG_PRINTLN("heading: " + String(heading_current));
                    double heading_discrepancy_new = diff_anglesf(heading_current, heading_desired);
                    heading_discrepancy_delta = heading_discrepancy_new - heading_discrepancy;
                    heading_discrepancy = heading_discrepancy_new;
                    // heading_discrepancy_sum += heading_discrepancy;
                    heading_discrepancy_sum = 0;
                    int pid_correction = round(0.01 * heading_discrepancy + 0.0001 * heading_discrepancy_delta + 0.01 * heading_discrepancy_sum);
                    DEBUG_PRINTLN("pid_correction: " + String(pid_correction));
                    motors_increment_velocity_left(0 - pid_correction);
                    motors_increment_velocity_right(pid_correction);
                }
            });

    
    loop_heading
        .when((void*)time_elapsed_ms_50)
        .then((void*)+[]() -> void {
                if (bno_connected) {
                    // ask how much time actually passed, independant of when clause
                    sensors_event_t event_magnet;
                    bno.getEvent(&event_magnet, Adafruit_BNO055::VECTOR_MAGNETOMETER);
                    float heading_measured = 180 * atan2(event_magnet.orientation.y, event_magnet.orientation.x) / PI;
                    heading_current = kalman_filter_heading.updateEstimateAngle(heading_measured);                    
                }
            });

    loop_turn_right
        .when((void*)time_elapsed_ms_2000)
        .then((void*)+[]() -> void {
                motors_set_velocity_left(-20);
                motors_set_velocity_right(-100);
            })
        .when((void*)time_elapsed_ms_2000)
        .then((void*)+[]() -> void {
                motors_set_velocity_left(100);
                motors_set_velocity_right(20);
            });

    loop_turn_left
        .when((void*)time_elapsed_ms_2000)
        .then((void*)+[]() -> void {
                motors_set_velocity_right(-1);
                motors_set_velocity_left(-100);
            })
        .when((void*)time_elapsed_ms_2000)
        .then((void*)+[]() -> void {
                motors_set_velocity_right(100);
                motors_set_velocity_left(1);
            });

    path_interruption_sequence[0] = (void*)+[]() -> void {
        // When the path is interrupted
        if (path_interrupted()) {
            // then stop and then set the desired heading to be 90 degrees to the right.
            // (The robot will begin to turn right).
            motors_set_velocity(0);
            heading_desired = heading_desired + 90.0;
            // change index!
            path_interruption_sequence_idx = 1;
        }
    };

    path_interruption_sequence[1] = (void*)+[]() -> void {
        // When the robot has completed its turn,
        if (diff_anglesf(heading_desired, heading_current) < 5.0) {
            // then go forth.
            motors_set_velocity(20);
            initial_time = millis();

            path_interruption_sequence_idx = 2;
        }
    };
   
    path_interruption_sequence[2] = (void*)+[]() -> void {
        // If an obstacle is detected, go to the beginning of the sequence.
        if (path_interrupted()) {
            path_interruption_sequence_idx = 0;
        }
        // When the 4 seconds has passed,
        if (millis() - initial_time > 4000) {
            // then stop and set the desired heading to be toward the original heading.
            motors_set_velocity(0);
            heading_desired = heading_original;
            path_interruption_sequence_idx = 3;
        }
    };

    path_interruption_sequence[3] = (void*)+[]() -> void {
        // When the robot has completed its turn,
        if (diff_anglesf(heading_desired, heading_current) < 5.0) {
            // then go forth
            motors_set_velocity(20);
            path_interruption_sequence_idx = 0;
        }
    };

    // ==============

    path_interruption_sequence_2[0] = (void*)+[]() -> void {
        // When the path is interrupted
        if (path_interrupted()) {
            // then stop and then set the desired heading to be 90 degrees to the right.
            // (The robot will begin to turn right).
            motors_set_velocity(0);
            heading_desired = heading_desired + 85.0;
            // change index!

            while (diff_anglesf(heading_desired, heading_current) > 10.0) {
                loop_turn_right();
                loop_heading();
                initial_time = millis();
            }

            motors_set_velocity(100);
            path_interruption_sequence_idx_2 = 1;
        }
    };

    path_interruption_sequence_2[1] = (void*)+[]() -> void {
        // If an obstacle is detected, go to the beginning of the sequence.
        if (path_interrupted()) {
            path_interruption_sequence_idx = 0;
        }
        // When the 4 seconds has passed,
        if (millis() - initial_time > 4000) {
            // then stop and turn toward the original heading.
            heading_desired = heading_original;
            
            while (diff_anglesf(heading_desired, heading_current) > 10.0) {                
                loop_turn_left();
                loop_heading();
            }
            
            motors_set_velocity(100);          
            path_interruption_sequence_idx_2 = 0;
        }
    };

    motors_init_pins();
    /* motors_set_velocity(100); */
//     motors_set_velocity_left(0);
//     motors_set_velocity_right(-100);
//     delay(1500);
//     motors_set_velocity_left(100);
//     motors_set_velocity_right(0);
//     delay(1500);
    motors_set_velocity(100);
}

void loop()
{
    // DEBUG_PRINTLN(String(sensor_left.distance()) + " " + String(sensor_right.distance()) + " " + String(sensor_left_chassis.distance()) + " " + String(sensor_right_chassis.distance()));
    
    // delay(500);
    // DEBUG_PRINTLN("");
    // the old plan:
    /* loop_pid_heading(); */
    // ((void (*)())(path_interruption_sequence[path_interruption_sequence_idx]))();

    // the current plan:
    loop_heading();
  
    DEBUG_PRINT(heading_current);
    DEBUG_PRINTLN(" " + String(heading_desired));
    ((void (*)())(path_interruption_sequence_2[path_interruption_sequence_idx_2]))();

    // testing:
    /* loop_turn_right(); */
}

NON_ARDUINO_MAIN
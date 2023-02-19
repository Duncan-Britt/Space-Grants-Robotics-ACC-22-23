#include "SharpIR.h"
#include "motors.h"
#include "pose.h"
#include "async.h"
#include "debug.h"
#include "utility.h"

#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "imumaths.h"

#include "SimpleKalmanFilter.h"

SimpleKalmanFilter kalman_filter_heading(2, 2, 0.01);
SimpleKalmanFilter kalman_filter_acceleration(2, 2, 0.01);

// ------
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/* #include <Wire.h> */
/* #include <Adafruit_Sensor.h> */
/* #include <Adafruit_BNO055.h> */

#ifdef DEBUG
#include "test.h" // Used for testing
#endif//DEBUG

// #include <Geometry.h> // https://github.com/tomstewart89/Geometry => Example use: https://github.com/tomstewart89/Geometry/blob/master/examples/HowToUse/HowToUse.ino

// using namespace Geometry;
// using namespace BLA;
// ^^^ could be used for linear algebra stuff?

//   _        _  _      ___  _        
//  / \ \  / |_ |_) \  / |  |_ \    / 
//  \_/  \/  |_ | \  \/ _|_ |_  \/\/  
// 
// The actions of this robot are modelled as a few loops that run in parrallel.
// Loops:
// 1. Every so often, (50 ms?) the robot should check its sensors and update the variables that model the
//    current state of the robot. 
// 2. In the beginning there is a long term goal pose for the robot. At any given time, the plan of action 
//    for the robot is stored in a queue of major poses. In the interest of conserving limited memory, transitions
//    between major poses are not stored in advance and are computed just in time. The robot is always trying to 
//    achieve its next pose. Once it is within a certain threshold of discrepancy, it can move on to the next pose.
// 3. Path planning, therefore, should consist of updating the sequence of desired poses for the robot
//    whenever new obstacles are encountered.
// 4. A PID algorithm (unimplemented 12/19/2022) will be used to correct for the discrepancy between the 
//    current pose and the desired pose. Or rather, 2 PID algorithms. One for rotational discrepancy and 
//    one for translational discrepancy.

// MODULES: (Updated Jan. 1)
// _______________________________________________________________________________________
// async.h: 
// ========
// There is no multi-threading/multi-process support on the Arduino, so this code implementes an interface 
// for defining distinct loops that will run asynchronously and independently of one another.

// Grid.h & Grid.cpp 
// ===================
// Defines an interface for plotting a path forward and avoiding obstacles. The territory is represented as
// an occupancy grid. The A* (star) path finding algorithm is used to search for the shortest path to the
// target destination.

// motors.h & motors.cpp
// =====================
// Defines an interface for incrementing and decrementing the velocity of the wheels.

// pose.h
// ======
// Implements some data types and functionality associated with the location and orientation of the robot, as
// well as a queue type to keep track of the poses in the planned path.

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

// Define model and input pin for IR sensor:
#define IRPin A0
#define MODEL 1080
// Create a new instance of the SharpIR class:
SharpIR sensor_IR = SharpIR(IRPin, MODEL);
// querry IR sensor data: sensor_IR.distance();
// -> returns an int type value that is the distance in centimeters from the sensor and the object in front of it.

Vec2Df final_position = { .x = 1000, .y = 0 };
Vec2DfQueue position_queue;
PoseF pose_array[2];
uint8_t idx_pose_array = 0;
bool bno_connected = false;
Vec2Df velocity_current = { .x = 0, .y = 0 };
PoseF pose_current = { .translation = { .x = 0, .y = 0 }, .rotation = 0.0 };

const float elapsed_s = 0.01;

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

#define THRESHOLD_MINOR_ROT 0.03 // radians
bool pose_achieved()
{ 
    switch (idx_pose_array) {
    case 0: {
      // To transition to the first pose, the robot only need rotate. Therefore, to confirm that it has achieved its goal,
      // we simply ask wether the discrepancy is within a certain threshold.
        return abs(pose_current.rotation - pose_array->rotation) < THRESHOLD_MINOR_ROT;
    }
    case 1: {
      // And now for the tricky bit
      // When the robot attempts to maintain its orientation and move straight ahead, to the second pose,
      // it may do so imperfectly. It may veer off course some what if it fails to keep totally straight.
      // We need a way to tell whether the robot has gone far enough in the direction of the last pose
      // without considering whether it ends up close to the desired destination, in case it has veered off course.
      // Vectors a & b describe the position of the robot at the first and second poses.      
        Vec2Df* a = (Vec2Df*) pose_array;
        Vec2Df* b = (Vec2Df*) (pose_array + 1);
      // Imagine a line going from a to b. Now imagine a line perpendicular to that line passing through b, called f(x).
      // We might ask whether the robot and point a are on opposite sides of this line and say that if so, it has gone far enough.

        // slope of line from a to b:
        double m = (double)(b->y - a->y) / (double)(b->x - a->x);
        m = (m == 0) ? 0.01 : 0; // TO PREVENT DIVIDE BY 0 ERROR
        // f(x) describes a line perpendicular to a line between the points a & b, passing through b;
        auto f = [b, m](double x) -> double { return (double)b->y - (x - (double)b->x) / m; };

      // But there's a problem: If the robot goes just up to the line and stops just before crossing it, such that it's at
      // the right pose, we would never know that the robot has gone far enough, and the robot would just sit there.
      // To account for this, we want some leeway. Instead of using f(x) which passes through b, we want to use a line
      // parallel to f(x) that's a little bit closer to a.

      // An effort was made to arrive at an exact answer: https://www.desmos.com/calculator/rglopobck7
      // If you move the sliders, you'll notce g(x) disappears, meaning there's no real solution.

      // Instead, I think this approximation will be good enough: => https://www.desmos.com/calculator/cwspo2d80p
      // and faster to compute!

        const double k = 5.0; // (cm) THIS CONSTANT IS SOMEWHAT ARBITRARY AND SHOULD BE TUNED.
        if (m > -0.5) {
            // g(x) = f(x + k) - k
            auto g = [f, k](double x) -> double { return f(x + k) - k; };
            // (yc > g(xc)) != (ya > g(xa))
            return ((double)pose_current.translation.y > g(pose_current.translation.x)) != ((double)a->y > g(a->x));
        }
        else {
            // g(x) = f(x) + k
            auto g = [f, k](double x) -> double { return f(x) + k; };
            // (yc > g(xc)) != (ya > g(xa))
            return ((double)pose_current.translation.y > g(pose_current.translation.x)) != ((double)a->y > g(a->x));            
        }
    }
    }
}

bool position_achieved()
{ // returns true if pose_current is close enough to the next goal pose (position_queue.front()).
    return pose_achieved() && idx_pose_array == 1;
}

//  ___       _        
//   |  _    | \  _  o 
//   | (_)   |_/ (_) o 

// Tests!

// Implement PID -> Test with arbitrary paths.

// Consider the case where an obstacle prevents the robot from rotating in place.
       // Back up to a pose in direction of previous position
       // (Remember newfound obstacles)

// Figure out how to translate between poses in the global and local reference frame => Given a position (x,y) relative to 
// the robot and the current pose of the robot in a global coordinate system, (x, y, θ), convert the coordinates of the
// position to a coordinate point in the global coordinate system.

// Write a function which answers the question of whether the robots path is interrupted by some obstacle - such as
// a wall, mound, or a ditch - given the current state of the robot (latest sensor data, current pose, and planned path).
// - Given that the robot is doing tank turns, we don't have to worry about obstacles while turning. We're looking for
// obstacles that lie in front (or behind?) of the robot, but only when the robot is going forward.

// Implement a function which, given a grid and path on the grid, removes the unnecessary path nodes.
// This is to say, if a series of nodes on the path are along a straight line, only the ends of the line
// should remain.

// Consider sampling based methods for path planning e.g. Probabilistic Roadmaps (PRMs)
// Randomly Expanding Random Trees (RRT or RRT*) https://youtu.be/XIBP0vCKl78?t=2307
// Potential Fields
// Can implement multiple path finding algorithms and be able to test among them. RRT* looks promising
// https://ieeexplore.ieee.org/document/6617944 -> this is about an RRT implementation for limited memory consumption 
// tutorial explains RRT: https://www.youtube.com/watch?v=OXikozpLFGo

// Consider non uniform cell representation in occupancy grid:
// https://ieeexplore.ieee.org/document/9304571
// This could save a lot of memory.

uint8_t rn = 0; // most recent distance reading for downward (hole) sensor
uint8_t rm = 0; // second most recent distance reading for downward (hole) sensor
uint8_t rl = 0; // third most recent distance reading for downard (hole) sensor
const uint8_t r_threshold = 100; // WHAT SHOULD THIS BE? WE DON'T KNOW YET
const uint8_t dr_threshold = 15; // WHAT SHOULD THIS BE? WE DON'T KNOW YET
const uint8_t r_expected = 80; // WHAT SHOULD THIS BE? WE DON'T KNOW YET
bool bumped = false;

bool path_interrupted_by_hole()
{
    return rn > r_threshold &&
        (rn - rm > dr_threshold &&
         rn - rl > dr_threshold);
}

bool path_interrupted()
{ // returns true if there is an obstacle in the way.
    if (bumped) {
        bumped = false;
        return true;
    }

    return path_interrupted_by_hole();
}

void stop_and_update_obstacles() {
    position_queue.clear();
    // set the next pose close by.
    // ...
    /* Pose_enqueue_transition(&pose_current, &desired_vec2d, pose_array); */
    idx_pose_array = 0;
}
// clears the pose queue and sets the next pose to be at or near the current position (pose_current)
// by enqueuing the pose queue (pose_array).

void reroute() {}
// Ultimate goal of this function is to fill up the pose queue with a new set up poses that describe the best guess at a
// path forward. Could use grid_find_path() declared in "astar.h". This would require translating between Vec2D coordinates (cm?)
// and the grid in astar.h. Doing so requires choosing how the grid should be scaled to represent space, deciding a pose for the grid,
// and then initializing the grid based on the best knowledge of where obstacles exist.

void pose_current_update() {}
// Use a combination of 
// - Positioning beacon system
// - gyroscope,
// - kinematic model based odometry (wheel encoders)
// - compass
// - ???
// to ascertain the location and orientation of the robot.
// https://www.youtube.com/watch?v=ZW7T6EFyYnc
// Modify pose_current accordingly.

/* Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); */

/* unsigned long T = millis(); // current time */
/* float DT; // delay between two updates of the filter */

void update_sensor_data() 
{
    if (bno_connected) {
        sensors_event_t event_euler; 
        bno.getEvent(&event_euler);
        sensors_event_t event_magnet;
        bno.getEvent(&event_magnet, Adafruit_BNO055::VECTOR_MAGNETOMETER);
        sensors_event_t event_gyro;
        bno.getEvent(&event_gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
        sensors_event_t event_accel;
        bno.getEvent(&event_accel, Adafruit_BNO055::VECTOR_LINEARACCEL);

        /* velocity_current.x += event_accel.acceleration.x > 0 ? 96 * event_accel.acceleration.x : 100 * event_accel.acceleration.x; */
        /* velocity_current.y += 100 * event_accel.acceleration.y; */

        /* pose_current.translation.x += velocity_current.x; */
        /* pose_current.translation.y += velocity_current.y; */

        float heading_measured = 180 * atan2(event_magnet.orientation.y, event_magnet.orientation.x) / PI;
        DEBUG_PRINT("heading: ");
        DEBUG_PRINT(heading_measured);
        DEBUG_PRINT("\tk: ");
        DEBUG_PRINTLN(kalman_filter_heading.updateEstimateAngle(heading_measured));

        // float accel_measured = event_accel.acceleration.x * 100;
        // DEBUG_PRINT("ax: ");
        // DEBUG_PRINT(accel_measured);
        // DEBUG_PRINT("\tkax: ");
        // float kax = kalman_filter_acceleration.updateEstimate(accel_measured);
        // DEBUG_PRINT(kax);
        // DEBUG_PRINT("\tvx: ");
        // DT = millis() - T;
        // T = millis();
        // velocity_current.x += kax * DT;
        // DEBUG_PRINTLN(velocity_current.x);



        /* Display the floating point data */
        // DEBUG_PRINT("Euler X: ");
        // DEBUG_PRINT(event_euler.orientation.x);
        /* DEBUG_PRINT("\tMagnet X: "); */
        /* DEBUG_PRINT(event_magnet.orientation.x); */
        /* DEBUG_PRINT("\tAx: "); */
        /* DEBUG_PRINT(event_accel.acceleration.x * 100); */
        /* DEBUG_PRINT("\tAy: "); */
        /* DEBUG_PRINT(event_accel.acceleration.y * 100); */
        /* DEBUG_PRINT("\tAccel Z: "); */
        /* DEBUG_PRINT(event_accel.acceleration.z); */
        /* DEBUG_PRINT("\t Vx: "); */
        /* DEBUG_PRINT(velocity_current.x); */
        /* DEBUG_PRINT("\t Vy: "); */
        /* DEBUG_PRINT(velocity_current.y); */
        /* DEBUG_PRINT("\t X: "); */
        /* DEBUG_PRINT(pose_current.translation.x); */
        /* DEBUG_PRINT("\t Y: "); */
        /* DEBUG_PRINT(pose_current.translation.y);         */
        /* DEBUG_PRINT("\tGyro X: "); */
        /* DEBUG_PRINT(event_gyro.gyro.x); */
        /* DEBUG_PRINT("\tGyro Y: "); */
        /* DEBUG_PRINT(event_gyro.gyro.y); */
        /* DEBUG_PRINT("\tGyro Z: "); */
        /* DEBUG_PRINT(event_gyro.gyro.z);  */
        
        // DEBUG_PRINT("\tatan2(y,x): ");
        // DEBUG_PRINTLN(180.0 * atan2(event_magnet.orientation.y, event_magnet.orientation.x) / PI);

        /* DEBUG_PRINTLN(diff_angles(350, 10)); */
        /* DEBUG_PRINTLN(diff_angles(350, 10)); */
        
    //     DEBUG_PRINT("\tY: ");
    //     DEBUG_PRINT(event.orientation.y);
    //     DEBUG_PRINT("\tZ: ");
    //     DEBUG_PRINT(event.orientation.z);
        /* DEBUG_PRINTLN(""); */
    }


    // if (sensor_IR.distance() < 80) {
    //   motors_set_velocity(0);
    // } else {
    //   motors_set_velocity(100);
    // }
    // DEBUG_PRINTLN(sensor_IR.distance());
      
    /* sensors_event_t orientationData , linearAccelData; */
    /* bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); */
    /* //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE); */
    /* bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); */

    /* //velocity = accel*dt (dt in seconds) */
    /* //position = 0.5*accel*dt^2            //sample rate = 50 */
    /* double ACCEL_VEL_TRANSITION =  (double)(50) / 1000.0; */
    /* double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION; */
    /* double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees */

    /* double xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x; */
    /* double yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y; */

    /* // velocity of sensor in the direction it's facing */
    /* double headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x); */

    /* DEBUG_PRINT("Heading: "); */
    /* DEBUG_PRINTLN(orientationData.orientation.x); */
    /* DEBUG_PRINT("Position: "); */
    /* DEBUG_PRINT(xPos); */
    /* DEBUG_PRINT(" , "); */
    /* DEBUG_PRINTLN(yPos); */
    /* DEBUG_PRINT("Speed: "); */
    /* DEBUG_PRINTLN(headingVel); */
    /* DEBUG_PRINTLN("-------"); */
}
// Query the sensors on the robot. Set global variables associated with sensor data according to the latest reading.

char pid_error_rotational(double rot_current, double rot_desired) {}
// ^^^ be careful of the difference in rotation (consider that the discrepancy between 7π/4 and 0 should be π/4, not 7π/4).
char pid_error_translational(Vec2Df pos_current, Vec2Df pos_desired) {}
// https://en.wikipedia.org/wiki/PID_controller
// https://www.youtube.com/watch?v=UR0hOmjaHp0
// https://www.youtube.com/watch?v=337Sp3PtVDc
// these return a number representing the magnitude and direction (+-) of the corrective measures needed to be taken
// to rectify the discrepancy between the desired pose and the current pose (pose_current).
// Should fall in the range [-100, 100]

// and many more...
//   _           ___       _        
//  |_ ._   _|    |  _    | \  _    
//  |_ | | (_|    | (_)   |_/ (_) o

void pid_correction_rotational() 
{   
    char pid_correction = pid_error_rotational(pose_current.rotation, (pose_array + idx_pose_array)->rotation); // positive correction means turn left
    motors_increment_velocity_left(0 - pid_correction);
    motors_increment_velocity_right(pid_correction);
}

void pid_correction_translational() 
{
    char pid_correction = pid_error_translational(pose_current.translation, (pose_array + idx_pose_array)->translation);
    motors_increment_velocity_left(pid_correction);
    motors_increment_velocity_right(pid_correction);
}

void update_perception_localize_and_pid()
{
    update_sensor_data();        // perception
    pose_current_update();       // localization
    pid_correction_rotational(); // motion control
    // Only perform translational correction if on the second pose
    if (idx_pose_array == 1) {
        pid_correction_translational();
    }
}

bool time_elapsed_ms_1000()
{
    static uint32_t prev_millis = millis();

    if (millis() - prev_millis >= 1000) {
         prev_millis = millis();
        return true;
    }

    return false;
}

/* AsyncLoop loop_obstacle; */
/* AsyncLoop loop_positions; */
/* AsyncLoop loop_poses; */
/* AsyncLoop loop_pid; */
AsyncLoop loop_pid_heading;
AsyncLoop loop_path_interrupted;

double heading_current;
double heading_desired;
double heading_original;

double heading_discrepancy = 0;
double heading_discrepancy_delta = 0;
double heading_discrepancy_sum = 0;

unsigned long initial_time = 0;
void* path_interruption_sequence[8];
uint8_t path_interruption_sequence_idx = 0;

void setup() 
{
    DEBUG_BEGIN(9600); // Needed to print to Serial Monitor.

    /* Initialise the sensor */
    if(!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      DEBUG_PRINTLN("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      // while(1);
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
    
    motors_init_pins();

    loop_pid_heading
        .when((void*)time_elapsed_ms_50)
        .then((void*)+[]() -> void {
                if (bno_connected) {
                    sensors_event_t event_magnet;
                    bno.getEvent(&event_magnet, Adafruit_BNO055::VECTOR_MAGNETOMETER);
                    float heading_measured = 180 * atan2(event_magnet.orientation.y, event_magnet.orientation.x) / PI;
                    heading_current = kalman_filter_heading.updateEstimateAngle(heading_measured);                    
                    double heading_discrepancy_new = diff_anglesf(heading_current, heading_desired);
                    heading_discrepancy_delta = heading_discrepancy_new - heading_discrepancy;
                    heading_discrepancy = heading_discrepancy_new;
                    heading_discrepancy_sum += heading_discrepancy;
                    int pid_correction = round(1 * heading_discrepancy + 1 * heading_discrepancy_delta + 0.01 * heading_discrepancy_sum);
                    motors_increment_velocity_left(0 - pid_correction);
                    motors_increment_velocity_right(pid_correction);
                }                
            });

    // When an object is within 60 cm,
    path_interruption_sequence[0] = (void*)+[]() -> bool {
        return sensor_IR.distance() < 60;
    };
    // then stop and then set the desired heading to be 90 degrees to the right.
    // (The robot will begin to turn right).
    path_interruption_sequence[1] = (void*)+[]() -> void {
        motors_set_velocity(0);
        heading_desired = heading_desired + 90.0;
    };
    // When the robot has completed its turn,
    path_interruption_sequence[2] = (void*)+[]() -> bool {
        return diff_anglesf(heading_desired, heading_current) < 5.0;
    };
    // then go forth.
    path_interruption_sequence[3] = (void*)+[]() -> void {
        motors_set_velocity(50);
        initial_time = millis();
    };
    // If an obstacle is detected, go to the beginning of the sequence.
    // When the 4 seconds has passed, 
    path_interruption_sequence[4] = (void*)+[]() -> bool {
        if (sensor_IR.distance() < 60) {
            path_interruption_sequence_idx = 0;            
            return false;
        }
        
        return millis() - initial_time > 4000;
    };
    // then stop and set the desired heading to be toward the original heading.
    path_interruption_sequence[5] = (void*)+[]() -> void {
        motors_set_velocity(0);
        heading_desired = heading_original;
    };
    // When the robot has completed its turn,
    path_interruption_sequence[6] = (void*)+[]() -> bool {
        return diff_anglesf(heading_desired, heading_current) < 5.0;
    };
    // then go forth.
    path_interruption_sequence[7] = (void*)+[]() -> void {
        motors_set_velocity(50);
    };

    motors_set_velocity(50);
    
    /* loop_path_interrupted */
    /*     .when((void*)+[]() -> bool { */
    /*             return sensor_IR.distance() < 60; */
    /*         }).then((void*)+[]() -> void { */
    /*                 motors_set_velocity(0); */
    /*                 heading_desired = heading_desired + 90.0; */
    /*             }).when((void*)+[]() -> bool { */
    /*                     return diff_anglesf(heading_desired, heading_current) < 5.0; */
    /*                 }).then((void*)+[]() -> void { */
    /*                         motors_set_velocity(50); */
    /*                         initial_time = millis(); */
    /*                     }).when((void*)+[]() -> bool { */
    /*                             // what if we run into an obstacle at this point? */
    /*                             // if sensor_ir.distance() < 60, go to first then(); */
    /*                             return millis() - initial_time > 4000; */
    /*                         }).then((void*)+[]() -> void { */
    /*                                 motors_set_velocity(0); */
    /*                                 heading_desired = heading_original; */
    /*                             }).when((void*)+[]() -> bool { */
    /*                                     return diff_anglesf(heading_desired, heading_current) < 5.0; */
    /*                                 }).then((void*)+[]() -> void { */
    /*                                         motors_set_velocity(50); */
    /*                                     }); */

        
    /* PoseF_enqueue_transition(&pose_current, &final_position, pose_array); */
    /* DEBUG_PRINTLN(F("\n")); */

    /* loop_obstacle */
    /*     .when((void*)path_interrupted) */
    /*     .then((void*)stop_and_update_obstacles) */
    /*     .when((void*)+[]() -> bool { */
    /*             return idx_pose_array == 1 && pose_achieved(); */
    /*         }) */
    /*     .then((void*)reroute); */

    /* loop_positions */
    /*     .when((void*)position_achieved) */
    /*     .then((void*)+[]() -> void { // Enqueue poses, dequeue position. */
    /*             if (!(position_queue.empty())) { */
    /*                 PoseF_enqueue_transition(&pose_current, position_queue.front(), pose_array); */
    /*                 idx_pose_array = 0; */
    /*                 position_queue.dequeue(); */
    /*             } */
    /*         }); */

    /* // In between two positions in the position_queue, there are poses */
    /* // which must be traversed */
    /* loop_poses // go to second pose when first is achieved */
    /*     .when((void*)pose_achieved) */
    /*     .then((void*)+[]() -> void { idx_pose_array = 1; }); */
    
    /* loop_pid */
    /*     .when((void*)time_elapsed_ms_50) */
    /*     .then((void*)update_perception_localize_and_pid); */

    /* DEBUG_PRINTLN_TRACE(freeRam()); */
    /* // test_IDA_star(); */
    /* test_a_star(); */
    /* DEBUG_PRINTLN_TRACE(freeRam()); */
    /* // test_motors(); */
}

void loop()
{
    loop_pid_heading();
    uint8_t idx = path_interruption_sequence_idx;
    if (idx % 2 == 0) {
        if ( ((bool (*)())(path_interruption_sequence[idx]))() ) {
            ((void (*)())(path_interruption_sequence[idx + 1]))();
            path_interruption_sequence_idx = (idx + 2) % 8;
        }           
    }
    
    /* loop_obstacle(); */
    /* loop_positions(); */
    /* loop_poses(); */
    /* loop_pid(); */
}

NON_ARDUINO_MAIN
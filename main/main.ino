#include <SharpIR.h>
#include "astar.h"
#include "motors.h"
#include "pose.h"
#include "async.h"
#include "test.h" // Used for testing/debugging
// #include <Geometry.h> // https://github.com/tomstewart89/Geometry => Example use: https://github.com/tomstewart89/Geometry/blob/master/examples/HowToUse/HowToUse.ino

// using namespace Geometry;
// using namespace BLA;

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

// HEADER FILES:
// _______________________________________________________________________________________
// async.h: 
// ========
// There is no multi-threading/multi-process support on the Arduino, so this code implementes an interface 
// for defining distinct loops that will run asynchronously and independently of one another.

// astar.h & astar.cpp 
// ===================
// Defines an interface for plotting a course through a cartesian grid containing open and blocked off spaces
// using the A* (star) path finding algorithm. Some work needs to be done to translate between traditional 
// coordinates and the grid defined in these files.

// motors.h & motors.cpp
// =====================
// Defines an interface for incrementing and decrementing the velocity of the wheels.

// pose.h
// ======
// Implements some data types and functionality associated with the location and orientation of the robot, as
// well as a queue type to keep track of the poses in the planned path.

// test.h
// ======
// A place to store code for testing and debugging.

// MISC NOTES:
// _______________________________________________________________________________________
// The code in the setup() function is run when the robot is turned on.
// Once the execution is finished, the loop() function runs until the robot is shut off.

//   _            _                         
//  |_ ._   _|   / \     _  ._   o  _       
//  |_ | | (_|   \_/ \/ (/_ | \/ | (/_ \/\/ 

// Define model and input pin for IR sensor:
#define IRPin A0
#define MODEL 1080
// Create a new instance of the SharpIR class:
SharpIR sensor_IR = SharpIR(IRPin, MODEL);
// querry IR sensor data: sensor_IR.distance();
// -> returns an int type value that is the distance in centimeters from the sensor and the object in front of it.

Pose pose_current = { .translation = { .x = 0, .y = 0 }, .rotation = 0.0 };
Pose pose_final = { .translation = { .x = 1000, .y = 0 }, .rotation = 0.0 };
PoseQueue pose_queue_major;
Pose pose_array_minor[3];
uint8_t idx_pose_array_minor = 0;

bool time_elapsed_ms_50() {
    static uint32_t prev_millis = millis();

    if (millis() - prev_millis >= 50) {
         prev_millis = millis();
        return true;
    }

    return false; 
}

//  ___       _        
//   |  _    | \  _  o 
//   | (_)   |_/ (_) o 

// Figure out how to translate between poses in the global and local reference frame

// Need a function to find the discrepancy between to angles. (Consider 2π - 0, and similar).

// Implement a function which, given a grid and path on the grid, removes the unnecessary path nodes.
// This is to say, if a series of nodes on the path are along a straight line, only the ends of the line
// should remain.

bool obstacle_detected() {}
// returns true if there is an obstacle.

void stop() {
    pose_queue_major.clear();
    // set the next pose close by.
    // ...
    /* Pose_enqueue_transition(&pose_current, &desired_pose, pose_array_minor); */
    idx_pose_array_minor = 0;
}
// clears the pose queue and sets the next pose to be at or near the current position (pose_current)
// by enqueuing the minor pose queue (pose_array_minor).

void reroute() {}
// Ultimate goal of this function is to fill up the pose queue with a new set up poses that describe the best guess at a
// path forward. Could use grid_find_path() declared in "astar.h". This would require translating between Vec2D coordinates (cm?)
// and the grid in astar.h. Doing so requires choosing how the grid should be scaled to represent space, deciding a pose for the grid,
// and then initializing the grid based on the best knowledge of where obstacles exist.

bool pose_next_achieved_major() {}
// returns true if pose_current is closed enough to the next goal pose (pose_queue_major.front()).

bool pose_next_achieved_minor() {}
// returns true if pose_current is close enought to the next minor pose:
// (pose_array_minor[idx_pose_array_minor])

void pose_current_update() {}
// Estimate the current position and orientation of the robot based on the latest sensor data.
// Modify pose_current accordingly.

void update_sensor_data() {}
// Query the sensors on the robot. Set global variables associated with sensor data according to the latest reading.

char pid_error_rotational(double rot_current, double rot_desired) {}
// ^^^ be careful of the difference in rotation (consider that the discrepancy between 2π and 0 should be 0, not 2π).
char pid_error_translational(Vec2D pos_current, Vec2D pos_desired) {}
// https://en.wikipedia.org/wiki/PID_controller
// these return a number representing the magnitude and direction (+-) of the corrective measures needed to be taken
// to rectify the discrepancy between the desired pose and the current pose (pose_current).
// Should fall in the range [-100, 100]

// and many more...
//   _           ___       _        
//  |_ ._   _|    |  _    | \  _    
//  |_ | | (_|    | (_)   |_/ (_) o

void update_state() 
{
    update_sensor_data();
    pose_current_update();
}

void pid_correction_rotational() 
{   
    char pid_correction = pid_error_rotational(pose_current.rotation, (pose_array_minor + idx_pose_array_minor)->rotation); // positive correction means turn left
    motors_increment_velocity_left(0 - pid_correction);
    motors_increment_velocity_right(pid_correction);
}

void pid_correction_translational() 
{
    char pid_correction = pid_error_translational(pose_current.translation, (pose_array_minor + idx_pose_array_minor)->translation);
    motors_increment_velocity_left(pid_correction);
    motors_increment_velocity_right(pid_correction);
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

AsyncLoop loop_state;
AsyncLoop loop_poses_major;
AsyncLoop loop_poses_minor;
AsyncLoop loop_obstacle; 
AsyncLoop loop_pid_rotational; 
AsyncLoop loop_pid_translational;

void setup() 
{
    Serial.begin(9600); // Needed to print to Serial Monitor.
    motors_init_pins();
    Pose_enqueue_transition(&pose_current, &pose_final, pose_array_minor);
    
    loop_state
        .when(time_elapsed_ms_50)
        .then(update_state);

    loop_poses_major
        .when(pose_next_achieved_major)
        .then([pose_current, pose_queue_major, pose_array_minor]() -> void { // Enqueue minor poses, dequeue major pose.
                if (!pose_queue_major.empty()) {
                    Pose_enqueue_transition(&pose_current, pose_queue_major.front(), pose_array_minor);
                    idx_pose_array_minor = 0;
                    pose_queue_major.dequeue();
                }
            });

    // In between two poses in the pose_queue_major, there are minor poses
    // which must be traversed
    loop_poses_minor
        .when(pose_next_achieved_minor)
        .then([pose_array_minor]() -> void { // dequeue minor pose
                if (idx_pose_array_minor < 2) {
                    ++idx_pose_array_minor;
                }
            });

    loop_obstacle
        .when(obstacle_detected)
        .then(stop)
        .when([pose_queue_major, pose_array_minor]() -> bool {
            return idx_pose_array_minor == 2 && pose_next_achieved_minor();
        })
        .then(reroute);

    loop_pid_rotational
        .when(time_elapsed_ms_50)
        .then(pid_correction_rotational);

    loop_pid_translational
        .when(time_elapsed_ms_50)
        .then(pid_correction_translational);
}

void loop() 
{
    loop_state();
    loop_poses_major();
    loop_poses_minor();
    loop_obstacle();
    loop_pid_rotational();
    loop_pid_translational();
}

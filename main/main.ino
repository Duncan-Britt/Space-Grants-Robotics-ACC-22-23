#include "SharpIR.h"
#include "motors.h"
#include "pose.h"
#include "async.h"
#include "debug.h"

#ifdef DEBUG
#include "test.h" // Used for testing
#endif//DEBUG

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

Pose pose_current = { .translation = { .x = 0, .y = 0 }, .rotation = 0.0 };
Vec2D vec2d_final = { .x = 1000, .y = 0 };
Vec2DQueue position_queue;
Pose pose_array[2];
uint8_t idx_pose_array = 0;

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
      // To transition to the first minor pose, the robot only need rotate. Therefore, to confirm that it has achieved its goal,
      // we simply ask wether the discrepancy is within a certain threshold.
        return abs(pose_current.rotation - pose_array->rotation) < THRESHOLD_MINOR_ROT;
    }
    case 1: {
      // And now for the tricky bit
      // When the robot attempts to maintain its orientation and move straight ahead, to the second minor pose,
      // it may do so imperfectly. It may veer off course some what if it fails to keep totally straight.
      // We need a way to tell whether the robot has gone far enough in the direction of the last minor pose
      // without considering whether it ends up close to the desired destination, in case it has veered off course.
      // Vectors a & b describe the position of the robot at the first and second poses.      
        Vec2D* a = (Vec2D*) pose_array;
        Vec2D* b = (Vec2D*) (pose_array + 1);
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

// Figure out how to translate between poses in the global and local reference frame
// Figure out the dimensions of the occupancy grid and the cells within which can be used to represent
// the surroundings.  Look in "astar.h".

// Need a function to find the discrepancy between 2 angles. (Consider 2π - 0, and similar).

// Implement a function which, given a grid and path on the grid, removes the unnecessary path nodes.
// This is to say, if a series of nodes on the path are along a straight line, only the ends of the line
// should remain.

bool obstacle_detected() {}
// returns true if there is an obstacle.

void stop() {
    position_queue.clear();
    // set the next pose close by.
    // ...
    /* Pose_enqueue_transition(&pose_current, &desired_vec2d, pose_array); */
    idx_pose_array = 0;
}
// clears the pose queue and sets the next pose to be at or near the current position (pose_current)
// by enqueuing the minor pose queue (pose_array).

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

void update_sensor_data() {}
// Query the sensors on the robot. Set global variables associated with sensor data according to the latest reading.

char pid_error_rotational(double rot_current, double rot_desired) {}
// ^^^ be careful of the difference in rotation (consider that the discrepancy between 7π/4 and 0 should be π/4, not 7π/4).
char pid_error_translational(Vec2D pos_current, Vec2D pos_desired) {}
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

void update_state() 
{
    update_sensor_data();
    pose_current_update();
}

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
AsyncLoop loop_positions;
AsyncLoop loop_poses;
AsyncLoop loop_obstacle; 
AsyncLoop loop_pid_rotational; 
AsyncLoop loop_pid_translational;

void setup() 
{
    DEBUG_BEGIN(9600); // Needed to print to Serial Monitor.
    motors_init_pins();
    Pose_enqueue_transition(&pose_current, &vec2d_final, pose_array);
    DEBUG_PRINTLN(F("\n"));
    test_a_star();

    
    loop_state
        .when((void*)time_elapsed_ms_50)
        .then((void*)update_state);

    loop_positions
        .when((void*)position_achieved)
        .then((void*)+[]() -> void { // Enqueue minor poses, dequeue major pose.
                if (!(position_queue.empty())) {
                    Pose_enqueue_transition(&pose_current, position_queue.front(), pose_array);
                    idx_pose_array = 0;
                    position_queue.dequeue();
                }
            });

    // In between two positions in the position_queue, there are poses
    // which must be traversed
    loop_poses
        .when((void*)pose_achieved)
        .then((void*)+[]() -> void { // dequeue minor pose
            if (idx_pose_array == 0) {
                ++idx_pose_array;
            }
        });

    loop_obstacle
        .when((void*)obstacle_detected)
        .then((void*)stop)
        .when((void*)+[]() -> bool {
            return idx_pose_array == 1 && pose_achieved();
        })
        .then((void*)reroute);

    loop_pid_rotational
        .when((void*)time_elapsed_ms_50)
        .then((void*)pid_correction_rotational);

    loop_pid_translational
        .when((void*)time_elapsed_ms_50)
        .then((void*)pid_correction_translational);

    DEBUG_PRINTLN_TRACE(freeRam());
    delay(1000);
    DEBUG_PRINTLN_TRACE(freeRam());
}

void loop() 
{
    loop_state();
    loop_positions();
    loop_poses();
    loop_obstacle();
    loop_pid_rotational();
    loop_pid_translational();
}

NON_ARDUINO_MAIN
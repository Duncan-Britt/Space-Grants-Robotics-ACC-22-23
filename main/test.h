#ifndef GUARD_test_h
#define GUARD_test_h

#ifdef ARDUINO
#include <Arduino.h>
#endif//ARDUINO

#include "grid.h"
#include "motors.h"
#include "pose.h"
#include "async.h"
#include "debug.h"

#define MAX_PATH_SIZE 16
#define DEBUG_GRID_STRING "\
........\n\
........\n\
...##...\n\
....#...\n\
....#...\n\
....####\n\
........\n\
........"
#define DEBUG_GRID_STRING_SIZE 72

void test_IDA_star()
{
    DEBUG_PRINTLN(F("beginning test"));

    Grid grid;

    char grid_string[DEBUG_GRID_STRING_SIZE] = DEBUG_GRID_STRING;
    char err = grid_init_str(grid_string, &grid);
    if (err) {
        DEBUG_PRINTLN(F("err"));
        free(grid.obstacles);
        return;
    }
    grid_print(&grid);
    DEBUG_PRINTLN(F(""));

    uint16_t path[MAX_PATH_SIZE]; // [0, 65535]
    uint8_t path_size = 0; // [0, 255]
    err = grid_find_path_IDA_star(&grid, 7, 63, path, &path_size, MAX_PATH_SIZE);
    switch (err) {
    case 0:
        grid_print_path(&grid, path, path_size);
        for (unsigned char i = 0; i < path_size; ++i) {
            DEBUG_PRINT(path[i]);
            DEBUG_PRINT(F(" "));
        }
        DEBUG_PRINTLN(F(""));
        break;
    case -1:        
        DEBUG_PRINTLN(F("Path exceeded max path size"));
        grid_print_path(&grid, path, path_size);
        for (unsigned char i = 0; i < path_size; ++i) {
            DEBUG_PRINT(path[i]);
            DEBUG_PRINT(F(" "));
        }
        DEBUG_PRINTLN(F(""));
        break;
    case -2:
        DEBUG_PRINTLN(F("Invalid Destination"));
        break;
    case -3:
        DEBUG_PRINTLN(F("Invalid start"));
        break;
    case -4:
        DEBUG_PRINTLN(F("Exceeded fCost threshold. Shouldn't be seeing this ever."));
    }
    free(grid.obstacles);
    DEBUG_PRINTLN(F("Freed"));  
}

void test_a_star() 
{
    DEBUG_PRINTLN(F("beginning test"));

    Grid grid;

    char grid_string[DEBUG_GRID_STRING_SIZE] = DEBUG_GRID_STRING;
    char err = grid_init_str(grid_string, &grid);
    if (err) {
        DEBUG_PRINTLN(F("err"));
        free(grid.obstacles);
        return;
    }
    grid_print(&grid);
    DEBUG_PRINTLN(F(""));

    uint16_t path[MAX_PATH_SIZE]; // [0, 65535]
    uint8_t path_size = 0; // [0, 255]
    err = grid_find_path_a_star(&grid, 7, 63, path, &path_size, MAX_PATH_SIZE);
    switch (err) {
    case 0:
        grid_print_path(&grid, path, path_size);
        for (unsigned char i = 0; i < path_size; ++i) {
            DEBUG_PRINT(path[i]);
            DEBUG_PRINT(F(" "));
        }
        DEBUG_PRINTLN(F(""));
        break;
    case 1:        
        DEBUG_PRINTLN(F("Path not found"));
        break;
    case -1:        
        DEBUG_PRINTLN(F("Path exceeded max path size"));
        grid_print_path(&grid, path, path_size);
        for (unsigned char i = 0; i < path_size; ++i) {
            DEBUG_PRINT(path[i]);
            DEBUG_PRINT(F(" "));
        }
        DEBUG_PRINTLN(F(""));
        break;
    case -2:
        DEBUG_PRINTLN(F("Invalid Destination"));
        break;
    case -3:
        DEBUG_PRINTLN(F("Invalid start"));
        break;
    case -4:
        DEBUG_PRINTLN(F("Exceeded max priority queue size"));
        break;
    case -5:
        DEBUG_PRINTLN(F("Exceeded max size for explored nodes array\n"));
    }
    // DEBUG_PRINTLN(freeRam());
    free(grid.obstacles);
    DEBUG_PRINTLN(F("Freed"));  
}

void test_motors_h() 
{
  
}

void test_pose_h() 
{
    const Vec2D a = { .x = 23, .y = -4 };
    const Vec2D b = { .x = -56, .y = 79 };
    Vec2D c;
    Vec2D_sub(&a, &b, &c);
    Vec2D_println(&c); // should print (79,-83);

    double angle = atan( ((double)c.y) / ((double)c.x) );
    DEBUG_PRINT(F("angle: "));
    Serial.println(angle); // should be -0.81 radians

    Pose current_pose = {
      .translation = { .x = 0, .y = 0 },
      .rotation = 0.0
    };

    Pose desired_pose = {
      .translation = { .x = 5, .y = 7 },
      .rotation = 1.0
    };

    /* Pose steps[3]; */
    /* Pose_set_transition(&current_pose, &desired_pose, steps); */
    /* for (Pose* p = &(steps[0]); p < steps+3; ++p) { */
    /*   Pose_println(p); */
    /* } */
}

// TEST EVENTS
void print_hello() 
{
    DEBUG_PRINTLN(F("Hi there"));  
}

void print_bye()
{
    DEBUG_PRINTLN(F("Bye now!"));
}

uint32_t prev_millis = 0;
uint32_t prev_millis2 = 0;

bool time_elapsed() 
{
    if (millis() - prev_millis > 3000) {
        prev_millis = millis();
        return true;
    }

    return false;
}

bool less_time_elapsed()
{
    if (millis() - prev_millis2 > 1000) {
        prev_millis2 = millis();
        return true;
    }

    return false;
}

// async.h test script
// uint32_t data = 0;
// AsyncLoop num_loop;

// void setup() 
// {
//     Serial.begin(9600); // Needed to print to Serial Monitor.
//     DEBUG_PRINTLN(F(""));

    // DEBUG_PRINTLN(F("ok"));
    // num_loop
    //   .when([data]() -> bool {
    //     return data == 73243;
    // }).then([data]() -> void {
    //     DEBUG_PRINT(F("Data:" );
    //     Serial.println(data);
    // }).when([data]() -> bool {
    //     return data == 0;
    // }).then([]() -> void {
    // }).when([data]() -> bool {
    //     return data == 89000;
    // }).then([data]() -> void {
    //     DEBUG_PRINT(F("D: "));
    //     Serial.println(data);
    // });
// }

// void loop() 
// {
    // data = (data + 1) % 100000;

    // if (data == 73243) {
    //   DEBUG_PRINTLN(F("if => 73243"));
    // }
    // num_loop();
// }

void test_vec2d_queue(Vec2DQueue vec2d_queue) {
    /* for (uint8_t i = 0; i < 100; ++i) { */
    /*     Pose a = { .translation = { .x = i, .y = (i / 2) }, .rotation = ((double)i / 10.0) }; */
    /*     pose_queue.enqueue(&a); */
    /* } */

    /* Serial.println(pose_queue.full() ? "full" : "not full")); */

    /* Vec2D_println(&(pose_queue.front()->translation)); */

    /* /\* pose_queue.println(); *\/ */
    /* Pose a = { .translation = { .x = 42, .y = (42 / 2) }, .rotation = ((double)42 / 10.0) }; */
    /* pose_queue.enqueue(&a); */
    /* pose_queue.println(); */
    
    /* for (uint8_t i = 0; i < 10; ++i) { */
    /*     Pose_print(pose_queue.front()); */
    /*     pose_queue.dequeue(); */
    /*     /\* pose_queue.println(); *\/ */
    /*     DEBUG_PRINTLN(F("")); */
    /* } */
    /* pose_queue.dequeue(); */
    /* /\* pose_queue.println(); *\/ */
    /* Serial.println(pose_queue.empty()); */

    /* Vec2D c = { .x = 0, .y = 0 }; */
    /* Vec2D d = { .x = 0, .y = 0 }; */

    /* Serial.println(c == d ? "c&d: Equal" : "c&d: not equal")); */

    Pose a = { .translation = { .x = 0, .y = 0 }, .rotation = 0 };
    Pose b = { .translation = { .x = 10, .y = -10 }, .rotation = 1.57 };

    /* pose_queue.enqueue_transition(&a, &b); */
    /* pose_queue.println(); */
    /* pose_queue.enqueue_transition(&a); */
    /* pose_queue.println(); */
}

#endif //GUARD_test_h





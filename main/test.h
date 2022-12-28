#ifndef GUARD_test_h
#define GUARD_test_h

#include <Arduino.h>
#include "astar.h"
#include "motors.h"

#define MAX_PATH_SIZE 16
#define DEBUG_GRID_STRING "\
........\n\
........\n\
....#...\n\
....#...\n\
....#...\n\
....#...\n\
........\n\
........"

void test_a_star() 
{
    Grid grid;

    char err = grid_init_str(DEBUG_GRID_STRING, &grid);
    if (err) {
        Serial.println("err");
        free(grid.obstacles);
        return;
    }
    // grid_print(&grid);
    // Serial.println("");

    unsigned int path[MAX_PATH_SIZE]; // [0, 65535]
    unsigned char path_size = 0; // [0, 255]
    err = grid_find_path(&grid, 7, 56, path, &path_size, MAX_PATH_SIZE);
    switch (err) {
    case 0:
        grid_print_path(&grid, path, path_size);
        for (unsigned char i = 0; i < path_size; ++i) {
            Serial.print(path[i]);
            Serial.print(" ");
        }
        Serial.println("");
        break;
    case 1:        
        Serial.println("Path not found");
        break;
    case -1:        
        Serial.println("Path exceeded max path size");
        grid_print_path(&grid, path, path_size);
        for (unsigned char i = 0; i < path_size; ++i) {
            Serial.print(path[i]);
            Serial.print(" ");
        }
        Serial.println("");
        break;
    case -2:
        Serial.println("Invalid Destination");
        break;
    case -3:
        Serial.println("Invalid start");
        break;
    }
    
    free(grid.obstacles);
    Serial.println("Freed");  
}

void test_motors_h() 
{
    motors_set_velocity_left_front(-10);
    motors_set_velocity_left_front(0);
}

#endif //GUARD_test_h


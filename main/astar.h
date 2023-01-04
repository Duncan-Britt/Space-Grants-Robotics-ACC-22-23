#ifndef GUARD_astar_h
#define GUARD_astar_h

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h> 
#include "debug.h"

typedef char Err;

typedef struct Grid {
    uint8_t* obstacles; // bit array
    unsigned char cols; // [0, 255]
    unsigned char rows; // [0, 255]
} Grid;

// 0's are open spaces and 1's are obstacles:
// _________________________________________
// 0 0 0 0 0
// 0 1 1 0 0
// 0 0 1 0 0
// 0 0 0 0 0

// The indeces of the above grid are laid out as follows:
// _____________________________________________________
//  0  1  2  3  4
//  5  6  7  8  9
// 10 11 12 13 14
// 15 16 17 18 19

// Euclidean distance beteen two spaces on grid, indicated by their indeces.
unsigned grid_distance(const Grid* grid, const unsigned int i, const unsigned int j);

// Implementation of the A* (star) algorithm for path finding.
// 4th argument should be an array of unsigned integers for storing the resulting path.
// 5th argument should be the address of a uint8_t for storing the resulting path size.
// Resulting path is in the form of grid indeces.
//
// Returns error codes in the event of failure, or 0 if success.
//  0: Success!
//  1: No path was found.
// -1: Path exceeded max path size.
// -2: Invalid target destination. The grid indicates there is an obstacle in the way.
// -3: Invalid start destination. The grid indicates there is an obstacle in the way. 
Err grid_find_path(const Grid* grid, const uint16_t start, const uint16_t dest, uint16_t* path, uint8_t* path_size, const uint8_t max_path_size);

// Predicate returns true if there is an obstacle at the given index.
bool grid_obstacle_at(const Grid* grid, size_t idx);

// Convert from grid index to (x,y) where the origin is at i = 0.
// All values for x and y >= 0.
void grid_idx_to_cartesian(const Grid* grid, const unsigned int i, int* x, int* y);

// Pretty Print frunctions for debugging
#ifdef DEBUG
    void grid_print(const Grid* grid);
    void grid_print_mark(const Grid* grid, const size_t marked);
    void grid_print_path(const Grid* grid, const unsigned int* path, const unsigned char path_size);

    // Useful for debugging: Initialize grid with string.
    // Currently grid->obstacles is malloced, and so needs to be freed.
    Err grid_init_str(char* s, Grid* grid);
    // EXAMPLE INITIALIZATION STRING:
    // #define DEBUG_GRID_STRING "\
    // ........\n\
    // ........\n\
    // ...##...\n\
    // ....#...\n\
    // ....#...\n\
    // ....##..\n\
    // ........\n\
    // ........"
    // The #'s are obstacles and the .'s are open spaces
#endif

#endif//GUARD_astar_h

#ifndef GUARD_astar_h
#define GUARD_astar_h

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h> 

typedef char Err;

typedef struct Grid {
    char* obstacles; // bit array
    unsigned char cols; // [0, 255]
    unsigned char rows; // [0, 255]
} Grid;

unsigned grid_distance(const Grid* grid, const unsigned int i, const unsigned int j);
Err grid_find_path(const Grid* grid, const unsigned int start, const unsigned int dest, unsigned int* path, unsigned char* path_size, const unsigned char max_path_size);
unsigned char grid_obstacle_at(const Grid* grid, size_t idx);
void grid_print(const Grid* grid);
void grid_print_mark(const Grid* grid, const size_t marked);
void grid_print_path(const Grid* grid, const unsigned int* path, const unsigned char path_size);
Err grid_init_str(char* s, Grid* grid);
void grid_idx_to_cartesian(const Grid* grid, const unsigned int i, int* x, int* y);

#endif
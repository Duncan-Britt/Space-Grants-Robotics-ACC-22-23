#ifndef GUARD_position_h
#define GUARD_position_h

#include <Arduino.h>

class Position {
 public:
    
 private:
    float x;
    float y;
    float z;
    float angle; // From straight ahead at origin
};

void obstacle_handler() {
    map_obstacle();
    choose_path_of_least_resistance();
}

void map_obstacle() {
    Position lower_bound = find_lower_bound();
    Position upper_bound = find_upper_bound();
    Position left_bound = find_left_bound();
    Position right_bound = find_right_bound();
}

void find_left_bound() {
    turn_sensor_left();
    register_event_handler(edge_detected, left_edge_detected_handler);
}

void left_edge_detected_handler() {
    turn_sensor_right();
    register_event_handler(edge_detected, right_edge_detected_handler);
}
	
#endif

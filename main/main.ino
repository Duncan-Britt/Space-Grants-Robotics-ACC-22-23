#include "wheels.h"
#include <SharpIR.h> // https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/
#include <Geometry.h> // https://github.com/tomstewart89/Geometry => Example use: https://github.com/tomstewart89/Geometry/blob/master/examples/HowToUse/HowToUse.ino
#include "astar.h"

// using namespace Geometry;
// using namespace BLA;

// Define model and input pin:
#define IRPin A0
#define model 1080
// TESTING REPLIT CHANGE
#define PIN_LEFT_FW 8
#define PIN_LEFT_BW 9
#define PIN_RIGHT_FW 10
#define PIN_RIGHT_BW 11
#define PIN_WHEELS_SPEED 3
#define PIN_WHEELS_SPEED2 5

#define MAX_PATH_SIZE 12

void setup() {
   Serial.begin(9600); // Needed to print to Serial Monitor.

    // Translation p_drone = {1.5, 0, 0.5};
    // Serial << "distance from drone to ground station: " << Norm(p_drone) << "\n";
    // pinMode(PIN_LEFT_FW, OUTPUT);
    // pinMode(PIN_LEFT_BW, OUTPUT);
    // pinMode(PIN_RIGHT_FW, OUTPUT);
    // pinMode(PIN_RIGHT_BW, OUTPUT);
    // pinMode(PIN_WHEELS_SPEED, OUTPUT);
    // pinMode(PIN_WHEELS_SPEED2, OUTPUT);

    char grid_string[120] = "\
...#.........\n\
.........#...\n\
.........#...\n\
.........#...\n\
.######.##...\n\
......#.#....\n\
......#.#....\n\
.............";
    // char grid_string[50] = "\
..#.\n\
.##.\n\
.#..\n\
....";

    Grid grid;

    char err = grid_init_str(grid_string, &grid);
    if (err) {
        Serial.println("err");
        free(grid.obstacles);
        return;
    }
    // grid_print(&grid);
    // Serial.println("");

    unsigned int path[MAX_PATH_SIZE]; // [0, 65535]
    unsigned char path_size = 0; // [0, 255]
    err = grid_find_path(&grid, 0, 103, path, &path_size, MAX_PATH_SIZE);
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

// int distance_cm;
// Create a new instance of the SharpIR class:
// SharpIR mySensor = SharpIR(IRPin, model);

void loop() {
  // Serial.println(millis());
  // Serial << millis() << "\n";
  // delay(1000);
  // digitalWrite(PIN_LEFT_BW, LOW);
  // digitalWrite(PIN_RIGHT_BW, LOW);
  // digitalWrite(PIN_LEFT_FW, HIGH);
  // digitalWrite(PIN_RIGHT_FW, HIGH);
  // analogWrite(PIN_WHEELS_SPEED, 255);
  // analogWrite(PIN_WHEELS_SPEED2, 255);
  // delay(3000);

  //   // Get a distance measurement and store it as distance_cm:
  // distance_cm = mySensor.distance();

  // // Print the measured distance to the serial monitor:
  // Serial.print("Mean distance: ");
  // Serial.print(distance_cm);
  // Serial.println(" cm");

  // delay(1000);

  // static const Wheels wheels(PIN_LEFT_FW, PIN_LEFT_BW, PIN_RIGHT_FW, PIN_RIGHT_BW, PIN_WHEELS_SPEED, PIN_WHEELS_SPEED2);
  // // Test script {
  // wheels.go();
  // delay(3000);
  // wheels.reverse();
  // delay(3000);
  // wheels.turn_right();
  // delay(3000);
  // wheels.turn_left();
  // delay(3000);
  // wheels.stop();
  // delay(5000);
  // }
}

// pins 10 & 11 are connected to in 3 & in 4 which should control the right drive?
// 10 is right forward
// 11 is right backward

// 8 is left forward
// 9 is left backward

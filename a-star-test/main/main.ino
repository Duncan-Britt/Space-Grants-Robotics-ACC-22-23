#include <math.h>
// #include "vec.h"
#include "grid.h"
//#include "array.h"

// const Vec r0 = { .x = -5, .y = -5 };
// const Vec rf = { .x = 5, .y = 5 };
const Vec r0(-2, -2);
const Vec rf(1, 0);

void setup() {
    Serial.begin(9600);
    Serial.println("\nWE'RE HERE!");
    Grid grid;
    grid.debug();
    grid.print();
    Serial.println("\n");

    Astar as;
    const size_t maxPathSize = 50;
    size_t pathSize = 0;
    Vec path[maxPathSize];
    as.search(r0, rf, grid, path, pathSize, maxPathSize);

    Serial.println("Out of search func");
    grid.print(path, pathSize);
}

void loop() {
  // put your main code here, to run repeatedly:

}
#include "PriorityQueue-master/PriorityQueue.h"
#include <math.h>
// #include "vec.h"
#include "grid.h"
//#include "array.h"

float distance(const Vec&, const Vec&);
bool compareGridPoints(const Vec&, const Vec&);

// const Vec r0 = { .x = -5, .y = -5 };
// const Vec rf = { .x = 5, .y = 5 };
const Vec r0(-5, -5);
const Vec rf(5, 5);

PriorityQueue<Vec> OPEN = PriorityQueue<Vec>(compareGridPoints);
PriorityQueue<Vec> CLOSED = PriorityQueue<Vec>(compareGridPoints);

void setup() {
    Serial.begin(9600);
    Serial.println("WE'RE HERE!");
    Grid grid;
    grid.debug();
    grid.print();
    Serial.println(grid(-1,1));
    Serial.println(distance(r0, rf));

    Astar as;

    if (as.search(r0, rf, grid)) {
      Serial.println("in if block.");
    }
}

bool compareGridPoints(const Vec& a, const Vec& b) {
  // this isn't done right yet
  // the distance to r0 is only relevant at first
  // then the relevant distance changes (to the current position?)
    const float costAg = distance(a, r0);
    const float costAh = distance(a, rf);
    const float costBg = distance(b, r0);
    const float costBh = distance(b, rf);
    const float costAf = costAg + costAh;
    const float costBf = costBg + costBh;

    if (costAf == costBf) {
	    return costAh < costBh;
    }

    return costAf < costBf;
}

float distance(const Vec& a, const Vec& b) {
    return sqrt(
		powf(abs(a.x - b.x), 2.0) + powf(abs(a.y - b.y), 2.0)
		);
}

void loop() {
  // put your main code here, to run repeatedly:

}
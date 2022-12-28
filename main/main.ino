#include <SharpIR.h> // https://www.makerguides.com/sharp-gp2y0a21yk0f-ir-distance-sensor-arduino-tutorial/
#include <Geometry.h> // https://github.com/tomstewart89/Geometry => Example use: https://github.com/tomstewart89/Geometry/blob/master/examples/HowToUse/HowToUse.ino
#include "astar.h"
#include "motors.h"
#include "test.h" // Used for testing/debugging

// using namespace Geometry;
// using namespace BLA;

// Define model and input pin:
#define IRPin A0
#define model 1080

void setup() {
    Serial.begin(9600); // Needed to print to Serial Monitor.

    test_a_star();
    // test_motors_h();

    // motors_init_pins();
}

// int distance_cm;
// Create a new instance of the SharpIR class:
// SharpIR mySensor = SharpIR(IRPin, model);

void loop() {
  // Serial.println(millis());
  // Serial << millis() << "\n";
  // delay(1000);

  //   // Get a distance measurement and store it as distance_cm:
  // distance_cm = mySensor.distance();

  // // Print the measured distance to the serial monitor:
  // Serial.print("Mean distance: ");
  // Serial.print(distance_cm);
  // Serial.println(" cm");

  // delay(1000);
}

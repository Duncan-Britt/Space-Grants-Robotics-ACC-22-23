#include "wheels.h"

const int ledPin = 2; // temporary for testing

void setup() {
  Serial.begin(9600); // Needed to print to Serial Monitor.
  pinMode(ledPin, OUTPUT);    // { temporary
  digitalWrite(ledPin, HIGH); //   for testing }
  Serial.println("\nSetup complete.\n");
}

// Need to figure out what the actual pins will be.
#define PIN_LEFT_FW 2
#define PIN_LEFT_BW 4
#define PIN_RIGHT_FW 7
#define PIN_RIGHT_BW 8
#define PIN_WHEELS_SPEED 3

void loop() {
  static const Wheels wheels(PIN_LEFT_FW, PIN_LEFT_BW, PIN_RIGHT_FW, PIN_RIGHT_BW, PIN_WHEELS_SPEED);
  // Test script {
  wheels.go();
  delay(3000);
  wheels.reverse();
  delay(3000);
  wheels.turn_right();
  delay(3000);
  wheels.turn_left();
  delay(3000);
  wheels.stop();
  delay(5000);
  // }
}

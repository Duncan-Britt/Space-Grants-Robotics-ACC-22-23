#include "wheels.h"
#include <SharpIR.h>

#define PIN_LEFT_FW 8
#define PIN_LEFT_BW 9
#define PIN_RIGHT_FW 10
#define PIN_RIGHT_BW 11
#define PIN_WHEELS_SPEED 3
#define PIN_WHEELS_SPEED2 5

void setup() {
  Serial.begin(9600); // Needed to print to Serial Monitor.

  pinMode(PIN_LEFT_FW, OUTPUT);
  pinMode(PIN_LEFT_BW, OUTPUT);
  pinMode(PIN_RIGHT_FW, OUTPUT);
  pinMode(PIN_RIGHT_BW, OUTPUT);
  pinMode(PIN_WHEELS_SPEED, OUTPUT);
  pinMode(PIN_WHEELS_SPEED2, OUTPUT);
}

void loop() {
  digitalWrite(PIN_LEFT_BW, LOW);
  digitalWrite(PIN_RIGHT_BW, LOW);
  digitalWrite(PIN_LEFT_FW, HIGH);
  digitalWrite(PIN_RIGHT_FW, HIGH);
  analogWrite(PIN_WHEELS_SPEED, 255);
  analogWrite(PIN_WHEELS_SPEED2, 255);
  delay(3000);

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

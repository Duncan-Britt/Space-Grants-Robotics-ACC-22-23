#include "debug.h"

#ifdef ARDUINO

int freeRam()
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#else

int freeRam() { return 0; }
unsigned long millis() { return 0; }
int analogRead(int pin) { return 0; }
void analogWrite(int pin, int value) {}
int digitalRead(int pin, int value) { return 0; }
void digitalWrite(int pin, int value) {}
void pinMode(int x, int mode) {}

#endif

#include "debug.h"

#ifdef ARDUINO

#if defined(ARDUINO_AVR_UNO) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560)
int freeRam()
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#else
#include <malloc.h>
#include <stdlib.h>
#include <stdio.h>

extern char _end;
extern "C" char *sbrk(int i);

int freeRam()
{
    char *ramstart=(char *)0x20070000;
    char *ramend=(char *)0x20088000;
    char *heapend=sbrk(0);
    register char * stack_ptr asm ("sp");
    struct mallinfo mi=mallinfo();

    return stack_ptr - heapend + mi.fordblks;
}
#endif

#else

int freeRam() { return 0; }
unsigned long millis() { return 0; }
int analogRead(int pin) { return 0; }
void analogWrite(int pin, int value) {}
int digitalRead(int pin, int value) { return 0; }
void digitalWrite(int pin, int value) {}
void pinMode(int x, int mode) {}

#endif

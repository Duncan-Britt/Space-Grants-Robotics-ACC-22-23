#ifndef GUARD_debug_h
#define GUARD_debug_h

#include <Arduino.h>

#define DEBUG

#ifndef ARDUINO
#include <iostream>
class SerialT {
public:
    template<class T>
    void print(T a) const {
	      std::cout << a;
    }

    template<class T>
    void println(T a) const {
	      std::cout << a << std::endl;
    }    
};
const SerialT Serial;

#define LOW 0
#define HIGH 1
#define pinMode(x, y)
#define digitalWrite(x, y)
#define analogWrite(x, y)
#define OUTPUT 1
#define INPUT 0
#define freeRam()

#ifdef __APPLE__ 
#include <unistd.h>
// untested
#define delay(x) sleep((int)((float)x / 1000.0))
#endif//APPLE

#endif//ARDUINO

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLN_TRACE(str) \
        Serial.print(millis()); \
        Serial.print(": "); \
        Serial.print(__FUNCTION__); \
        Serial.print("() in "); \
        Serial.print(__FILE__); \
        Serial.print(':'); \ 
        Serial.print(__LINE__); \
        Serial.print(' '); \
        Serial.println(str)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLN_TRACE(x)
#endif//DEBUG

#endif//GUARD_debug_h

// note:
/* DEBUG_PRINT ("I'm here"); */
/* could get printed as: */
/* "my_program.c::myFn line 1451 @ 1121 milliseconds I'm here" */
/* by using FILE, FUNCTION, LINE and "millis" and a few prints. */

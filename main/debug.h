#ifndef GUARD_debug_h
#define GUARD_debug_h

int freeRam();

#ifdef ARDUINO

#include <Arduino.h>
#define NON_ARDUINO_MAIN

#else

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <cstdint>
#include <cstdint>

typedef uint8_t byte;
typedef uint16_t word;

class SerialT {
public:
    void begin(const int n) const {}
    
    template<class T>
    void print(T a) const { std::cout << a; }

    template<class T>
    void println(T a) const { std::cout << a << std::endl; }    
};
const SerialT Serial;

#define double float
#define LOW 0
#define HIGH 1
#define OUTPUT 1
#define INPUT 0
#define F(x) x
#define A0 14

unsigned long millis();
int analogRead(int pin);
void analogWrite(int pin, int value);
int digitalRead(int pin, int value);
void digitalWrite(int pin, int value);
void pinMode(int x, int mode);

#ifdef __APPLE__ 
#include <unistd.h>
// untested
#define delay(x) sleep((int)((float)x / 1000.0))
#endif//APPLE

#define NON_ARDUINO_MAIN \
    int main()           \
    {                    \
        setup();         \
        while (true) {   \
            loop();      \
        }                \
    }

#endif//ARDUINO

#define DEBUG

#ifdef DEBUG
    #define DEBUG_BEGIN(x) Serial.begin(x)
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

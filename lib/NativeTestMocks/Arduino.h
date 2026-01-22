#ifndef ARDUINO_H
#define ARDUINO_H
#include <cstdint>
#include <cstdio>
#include <string.h>
#include <chrono>
#include <stdarg.h>
#include <cmath>
#include "Wire.h"
#include "Print.h"
#ifdef WIN32
#include <windows.h>
#endif
#define SS 10 // random ass numbers lol

#define HIGH 1
#define LOW 0

#define INPUT 1
#define OUTPUT 0

#define LED_BUILTIN 13
#define BUILTIN_SDCARD 254

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Arduino helper functions
template<typename T>
T constrain(T x, T low, T high) {
    if (x < low) return low;
    if (x > high) return high;
    return x;
}

template<typename T, typename U, typename V>
T map(T x, U in_min, U in_max, V out_min, V out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint64_t millis();

void setMillis(uint64_t ms);

void resetMillis();

void delay(unsigned long ms);

void delay(int ms);

void digitalWrite(int pin, int value);

class Stream : public Print
{
public:
    void begin(int baud = 9600);
    void end();
    void clearBuffer();
    bool available();
    int read() { return -1; }  // Mock read - returns -1 (no data)
    int readBytesUntil(char i, char *buf, size_t s);
    size_t write(uint8_t b) override;
    operator bool() { return true; }

    char fakeBuffer[1000];
    int cursor = 0;
};

class SerialClass : public Stream
{
};

extern SerialClass Serial;

class CrashReportClass
{
public:
    explicit CrashReportClass() {}
    operator bool() const { return false; }
};
extern CrashReportClass CrashReport;

#endif
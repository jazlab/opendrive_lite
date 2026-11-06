// Desktop test doubles for the Arduino functions used by the real sketch.
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <string>

constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OUTPUT = 1;
constexpr int HEX = 16;

extern uint32_t testMillis;
extern uint32_t testMicros;
extern int testEnable;
extern unsigned int testStepPulses[7];

inline unsigned long millis() { return testMillis; }
inline unsigned long micros() { return testMicros; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) {
    if (pin == 0) testEnable = value;
    if (pin >= 1 && pin <= 6 && value == HIGH) ++testStepPulses[pin];
}
inline void delayMicroseconds(unsigned int delay) { testMicros += delay; }

struct TestSerial {
    std::string input;
    std::string output;
    void begin(unsigned long) {}
    int available() { return static_cast<int>(input.size()); }
    char read() { char c = input.front(); input.erase(0, 1); return c; }
    void print(const char* text) { output += text; }
    void print(char value) { output += value; }
    void println(unsigned int value, int) {
        char text[16];
        snprintf(text, sizeof(text), "%X\n", value);
        output += text;
    }
};
extern TestSerial Serial;

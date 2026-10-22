// Host stand-in for the parts of Arduino.h the library and PID_v1 use.
#ifndef FAKE_ARDUINO_H
#define FAKE_ARDUINO_H

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

typedef uint8_t byte;

// Test-controlled clock.
extern unsigned long fake_millis;
inline unsigned long millis() { return fake_millis; }

// Same macros as the AVR core, so abs() on a double behaves as on target.
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

class String : public std::string
{
public:
  String() {}
  String(const char *s) : std::string(s) {}
  String(const std::string &s) : std::string(s) {}
  String(int v) : std::string(std::to_string(v)) {}
  String(unsigned int v) : std::string(std::to_string(v)) {}
  String(long v) : std::string(std::to_string(v)) {}
  String(unsigned long v) : std::string(std::to_string(v)) {}
  String(double v, int decimals = 2)
  {
    char buf[32];
    snprintf(buf, sizeof buf, "%.*f", decimals, v);
    assign(buf);
  }
};

#endif

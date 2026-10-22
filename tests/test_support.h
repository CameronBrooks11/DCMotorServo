// Fake motor and encoder wiring shared by the host tests.
#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H

#include <cstdio>
#include <cstdlib>

#include <Arduino.h>

inline long fake_encoder = 0;
inline int16_t fake_motor_speed = 0;

inline void fakeMotorWrite(int16_t speed) { fake_motor_speed = speed; }
inline void fakeMotorBrake() { fake_motor_speed = 0; }
inline long fakeEncoderRead() { return fake_encoder; }
inline void fakeEncoderWrite(long position) { fake_encoder = position; }

inline void resetFakes()
{
  fake_millis = 0;
  fake_encoder = 0;
  fake_motor_speed = 0;
}

static int test_failures = 0;

#define CHECK(cond)                                                \
  do                                                               \
  {                                                                \
    if (!(cond))                                                   \
    {                                                              \
      fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
      test_failures++;                                             \
    }                                                              \
  } while (0)

#endif

// #19: a position error a multiple of 65,536 counts away is not read as
// inside the deadband where abs() takes an int. Built with FAKE_INT16_ABS,
// which models the megaavr core's abs().
#include <DCMotorServo.h>

#include "test_support.h"

int main()
{
  resetFakes();
  DCMotorServo servo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
  servo.moveTo(65541); // low 16 bits: 5, inside the default 30-count band
  // The PID re-arms on the first run() and drives on the second.
  for (int i = 0; i < 2; i++)
  {
    fake_millis += 50;
    servo.run();
  }
  CHECK(fake_motor_speed != 0);
  CHECK(!servo.finished());

  return test_failures ? 1 : 0;
}

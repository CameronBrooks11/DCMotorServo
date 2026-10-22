// #11: setAccuracy() keeps values above 255 instead of wrapping them.
#include <DCMotorServo.h>

#include "test_support.h"

int main()
{
  resetFakes();
  DCMotorServo servo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
  servo.setAccuracy(300); // wrapped to 44

  // 280 counts off is inside a 300-count band, so the motor stays off.
  servo.moveTo(280);
  for (int i = 0; i < 2; i++)
  {
    fake_millis += 50;
    servo.run();
    CHECK(fake_motor_speed == 0);
  }
  CHECK(servo.finished());

  return test_failures ? 1 : 0;
}

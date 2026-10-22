// Checks the harness itself: a position move drives toward the target and
// stops inside the accuracy band.
#include <DCMotorServo.h>

#include "test_support.h"

int main()
{
  resetFakes();
  DCMotorServo servo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);

  servo.moveTo(1000);
  fake_millis += 50;
  servo.run();
  CHECK(fake_motor_speed > 0);

  servo.moveTo(-1000);
  fake_millis += 50;
  servo.run();
  CHECK(fake_motor_speed < 0);

  fake_encoder = -990;
  fake_millis += 50;
  servo.run();
  CHECK(fake_motor_speed == 0);
  CHECK(servo.finished());

  return test_failures ? 1 : 0;
}

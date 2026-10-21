// #13: stop() drops the pending target, so the next run() holds where the
// motor stopped and a later speed loop doesn't inherit the old target.
#include <DCMotorTacho.h>

#include "test_support.h"

int main()
{
  resetFakes();
  {
    DCMotorServo servo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
    servo.moveTo(1000);
    fake_millis += 50;
    servo.run();
    fake_encoder = 200;

    servo.stop();
    CHECK(servo.getRequestedPosition() == 200);
    // The PID re-arms on the first run() and drives on the second.
    for (int i = 0; i < 2; i++)
    {
      fake_millis += 50;
      servo.run();
      CHECK(fake_motor_speed == 0);
    }
  }

  // Slice_DCMT's switch from position to speed mode.
  resetFakes();
  {
    DCMotorServo servo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
    DCMotorTacho tacho(&servo, 798, 5);
    servo.moveTo(20000);
    fake_millis += 50;
    servo.run();

    servo.stop();
    tacho.stop();
    tacho.setSpeedRPM(10);
    fake_millis += 5;
    tacho.run();
    long lead = servo.getRequestedPosition() - servo.getActualPosition();
    CHECK(labs(lead) < 100);
  }

  return test_failures ? 1 : 0;
}

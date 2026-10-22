// #13: setMaxLead() bounds how far the speed loop can drive the target ahead
// of a stalled shaft. It is off by default.
#include <DCMotorTacho.h>

#include "test_support.h"

// rpm for 3 s with the shaft held still; returns the final lead.
static long stalledLead(double rpm, long maxLead)
{
  resetFakes();
  DCMotorServo servo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
  DCMotorTacho tacho(&servo, 798, 5);
  tacho.setPIDTunings(0.15, 0.01, 0.01);
  if (maxLead > 0)
    tacho.setMaxLead(maxLead);
  tacho.setSpeedRPM(rpm);
  for (int i = 0; i < 3000; i++)
  {
    fake_millis += 1;
    tacho.run();
  }
  return servo.getRequestedPosition() - servo.getActualPosition();
}

int main()
{
  for (double rpm : {100.0, -100.0})
  {
    long capped = stalledLead(rpm, 200);
    long uncapped = stalledLead(rpm, 0);
    fprintf(stderr, "%+.0f RPM: lead with setMaxLead(200): %ld, without: %ld\n", rpm, capped, uncapped);
    CHECK(labs(capped) <= 200);
    CHECK(labs(uncapped) > 200);
    CHECK(capped * rpm > 0);
  }

  return test_failures ? 1 : 0;
}

// #13: starting the speed loop measures from the start, not from wherever
// the encoder was when the loop last ran.
#include <cmath>

#include <DCMotorTacho.h>

#include "test_support.h"

int main()
{
  resetFakes();
  DCMotorServo servo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
  DCMotorTacho tacho(&servo, 798, 5);

  // Position-mode travel with the speed loop idle.
  fake_millis += 1000;
  fake_encoder = 5000;
  servo.moveTo(5000);

  // Four counts in the first 5 ms sample.
  tacho.setSpeedRPM(10);
  fake_millis += 5;
  fake_encoder += 4;
  tacho.run();

  double expected = 4 / 798.0 * (60000.0 / 5);
  fprintf(stderr, "measured %.2f RPM, expected %.2f RPM\n", tacho.getMeasuredSpeedRPM(), expected);
  CHECK(std::fabs(tacho.getMeasuredSpeedRPM() - expected) < 1e-9);

  return test_failures ? 1 : 0;
}

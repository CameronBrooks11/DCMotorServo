// #18: getMeasuredSpeedRPM() keeps measuring while the setpoint is 0, so it
// reads 0 once the shaft stops, and the shaft's speed if it is turned.
#include <cmath>

#include <DCMotorTacho.h>

#include "test_support.h"

static void runFor(DCMotorTacho &tacho, unsigned long ms, long countsPerMs)
{
  for (unsigned long i = 0; i < ms; i++)
  {
    fake_millis += 1;
    fake_encoder += countsPerMs;
    tacho.run();
  }
}

int main()
{
  resetFakes();
  DCMotorServo servo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
  DCMotorTacho tacho(&servo, 798, 5);

  // Run at about 75 RPM (1 count/ms), then set 0 with the shaft stopped.
  tacho.setSpeedRPM(75);
  runFor(tacho, 100, 1);
  double running = tacho.getMeasuredSpeedRPM();
  fprintf(stderr, "running: %.2f RPM\n", running);
  CHECK(running > 70);

  tacho.setSpeedRPM(0);
  runFor(tacho, 100, 0);
  fprintf(stderr, "stopped: %.2f RPM\n", tacho.getMeasuredSpeedRPM());
  CHECK(tacho.getMeasuredSpeedRPM() == 0);

  // Turned by hand at 2 counts/ms with the setpoint still 0.
  runFor(tacho, 100, 2);
  double expected = 2 / 798.0 * 60000.0;
  fprintf(stderr, "turned: %.2f RPM, expected %.2f RPM\n", tacho.getMeasuredSpeedRPM(), expected);
  CHECK(std::fabs(tacho.getMeasuredSpeedRPM() - expected) < 1e-9);

  return test_failures ? 1 : 0;
}

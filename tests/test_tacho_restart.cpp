// #12: a speed restart after setSpeedRPM(0) must start the speed PID from
// rest, so its target steps match a freshly constructed tacho's.
#include <DCMotorTacho.h>

#include "test_support.h"

static const double CPR = 798;
static const unsigned long INTERVAL = 5;
static const int SAMPLES = 20;

static DCMotorServo *makeServo()
{
  DCMotorServo *servo = new DCMotorServo(fakeMotorWrite, fakeMotorBrake, fakeEncoderRead, fakeEncoderWrite);
  servo->setPIDTunings(0.15, 0.01, 0.01);
  servo->setMaxPWM(150);
  servo->setAccuracy(10);
  return servo;
}

// Runs one speed sample per INTERVAL with the shaft held still and records
// how far the speed loop moves the position target each sample.
static void recordSteps(DCMotorTacho &tacho, long *steps)
{
  for (int i = 0; i < SAMPLES; i++)
  {
    long before = tacho.getServo()->getRequestedPosition();
    fake_millis += INTERVAL;
    tacho.run();
    steps[i] = tacho.getServo()->getRequestedPosition() - before;
  }
}

int main()
{
  long fresh[SAMPLES];
  long restarted[SAMPLES];

  resetFakes();
  {
    DCMotorServo *servo = makeServo();
    DCMotorTacho tacho(servo, CPR, INTERVAL);
    tacho.setPIDTunings(0.15, 0.01, 0.01);
    tacho.setSpeedRPM(10);
    recordSteps(tacho, fresh);
  }

  resetFakes();
  {
    DCMotorServo *servo = makeServo();
    DCMotorTacho tacho(servo, CPR, INTERVAL);
    tacho.setPIDTunings(0.15, 0.01, 0.01);

    // Run at 5 RPM; the last sample sees one count, which is 15.04 RPM, and
    // the derivative term leaves the output near -31 counts.
    tacho.setSpeedRPM(5);
    for (int i = 0; i < 4; i++)
    {
      fake_millis += INTERVAL;
      tacho.run();
    }
    fake_encoder += 1;
    fake_millis += INTERVAL;
    tacho.run();

    tacho.setSpeedRPM(0);
    fake_millis += 100;
    tacho.run();

    tacho.setSpeedRPM(10);
    recordSteps(tacho, restarted);
  }

  for (int i = 0; i < SAMPLES; i++)
  {
    if (restarted[i] != fresh[i])
      fprintf(stderr, "sample %d: restarted step %ld, fresh step %ld\n", i, restarted[i], fresh[i]);
    CHECK(restarted[i] == fresh[i]);
  }

  return test_failures ? 1 : 0;
}

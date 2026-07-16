# Examples

The first two will walk you through configuring and calibrating the control objects for your specific driver and motor

1. servo_config --> procedure for tuning a DCMotorServo object.
2. tacho_config --> procedure for tuning a DCMotorTacho object.
3. usage_demos --> ready-to-run demos for specific driver hardware.

## Configuration workflow

Every config sketch reads its hardware setup and calibration values from a `baseSpecs.h` in its own folder (shown as a tab in the Arduino IDE): driver selection, pins, motor specs (PPR / gear ratio / fudge factor), and the values each step determines (PWM skip, accuracy, PID gains, speed interval). Work through the steps in order — `servo_config` 01→04, then `tacho_config` — updating `baseSpecs.h` with each result and **copying the file forward into the next step's folder**. When you finish, your `baseSpecs.h` is the complete spec sheet for your motor+driver: lift its values into your real project's wrappers and setup.

All example sketches use the [L298N](https://github.com/CameronBrooks11/L298N_Arduino) and/or [LMD18200](https://github.com/CameronBrooks11/lmd18200) driver libraries (example-only dependencies — the library itself needs neither). See [docs/getting-started.md](../docs/getting-started.md#example-driver-libraries) for installation.
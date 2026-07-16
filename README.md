# DCMotorServo

An Arduino library for closed-loop control of brushed DC motors with quadrature encoders — position control like a servo (without the angle limit), plus an optional cascaded speed loop. A continuation of [julester23/DCMotorServo](https://github.com/julester23/DCMotorServo), inspired by [AccelStepper](https://github.com/adafruit/AccelStepper).

## Features

- **Position control** (`DCMotorServo`): PID on encoder counts — `moveTo()`, `move()`, `finished()`, accuracy dead-band, PWM-skip stiction compensation
- **Speed control** (`DCMotorTacho`): cascaded RPM loop layered on the position loop — `setSpeedRPM()`
- **Extrema sensing** (v1.1.0): software travel limits, physical endstop callbacks, encoder-based stall detection (no extra hardware), and non-blocking homing — endstop or sensorless — with a dead-switch failsafe
- **Driver-agnostic**: no pin logic in the library. You supply four function pointers and any H-bridge or driver IC works:

| Pointer | Signature | Purpose |
| --- | --- | --- |
| `MotorWriteFunc` | `void fn(int16_t speed)` | Signed PWM to the driver |
| `MotorBrakeFunc` | `void fn()` | Engage the brake |
| `EncoderReadFunc` | `long fn()` | Current encoder count |
| `EncoderWriteFunc` | `void fn(long pos)` | Reset/zero the count |

## Dependencies

Declared in the manifests and resolved automatically by PlatformIO; via Library Manager for Arduino IDE:

- [PID](https://github.com/br3ttb/Arduino-PID-Library) by Brett Beauregard
- [Encoder](https://www.pjrc.com/teensy/td_libs_Encoder.html) by Paul Stoffregen

The bundled examples additionally use the [L298N](https://github.com/CameronBrooks11/L298N_Arduino) and [LMD18200](https://github.com/CameronBrooks11/lmd18200) driver libraries (example-only — your own wrappers replace them in real projects).

## Getting started

```cpp
DCMotorServo servo(motorWrite, motorBrake, encoderRead, encoderWrite);

void loop() {
    servo.run();            // call every iteration
    if (servo.finished()) { /* ... */ }
}
```

- [docs/getting-started.md](docs/getting-started.md) — installation, the function-pointer concept, minimal wiring examples
- [docs/tuning.md](docs/tuning.md) — CPR determination, PID and accuracy tuning stages
- [docs/api.md](docs/api.md) — full API reference
- [examples/](examples/README.md) — guided config procedures (`servo_config`, `tacho_config`) and ready-to-run `usage_demos`

## Reference hardware

Developed and tested with a 26PG-3429-19-EN gearmotor (encoder) on L298N and LMD18200 drivers; any driver/encoder combination works through the function-pointer interface.

## License

MIT — see [LICENSE](LICENSE).

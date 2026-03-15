# Tuning Guide

Tuning has three stages: determine your encoder's CPR, configure the position controller (`DCMotorServo`), then — if using speed control — configure the speed loop (`DCMotorTacho`).

The `examples/servo_config/` and `examples/tacho_config/` sketches are designed to walk through each stage interactively.

---

## Stage 1 — Determine CPR (Counts Per Revolution)

CPR is the number of encoder counts for one full output shaft revolution **after the gearbox**, including the quadrature ×4 factor and any empirical correction.

**Formula:**

```
CPR = PPR × GEAR_RATIO × 4 × EMPIRICAL_FUDGE_FACTOR
```

| Value                    | Source                                    |
| ------------------------ | ----------------------------------------- |
| `PPR`                    | Encoder datasheet (pulses per revolution) |
| `GEAR_RATIO`             | Motor/gearbox datasheet                   |
| `4`                      | Quadrature decoding multiplier (fixed)    |
| `EMPIRICAL_FUDGE_FACTOR` | Measured — see below (start at `1.0`)     |

**To measure the fudge factor:**

1. Mark the output shaft.
2. Command `moveTo(CPR)` (one revolution) with fudge = 1.0.
3. Measure the actual angle. If it turned 370° instead of 360°, fudge = 360/370 ≈ 0.973.
4. Iterate until one revolution is accurate.

Only needed for `DCMotorTacho`. `DCMotorServo` works purely in raw counts.

---

## Stage 2 — Tune the Position Loop (DCMotorServo)

### 2a — Find PWM Skip

`setPWMSkip(value)` sets the minimum PWM added to every non-zero output so the motor overcomes static friction and actually moves.

1. Open `examples/servo_config/01_pwm_skip/`.
2. Slowly increase `MIN_PWM` until the motor reliably starts from rest.
3. Typical range: 10–60 depending on motor and gearbox.

### 2b — Set Accuracy

`setAccuracy(counts)` is the dead-band: when `|setpoint - encoder| < accuracy` the motor is considered done and PWM drops to zero.

- Too small → motor hunts (oscillates around target).
- Too large → coarse positioning.
- Start with ~1–2% of CPR. Tighten after PID is tuned.

### 2c — Tune PID Gains

Use `examples/servo_config/03_tune_servo_pid/` with the Arduino Serial Plotter (call `servo.getSerialPlotter()` every ~20 ms).

**Procedure:**

| Step | Action                                                                                  |
| ---- | --------------------------------------------------------------------------------------- |
| 1    | Set Ki = 0, Kd = 0. Increase Kp until the motor reaches the target without oscillating. |
| 2    | If there is steady-state error, increase Ki in small steps.                             |
| 3    | If the motor overshoots, increase Kd to dampen.                                         |
| 4    | Iterate: small Kd increase often lets you raise Kp further.                             |

**Typical starting point:** `Kp=0.15, Ki=0.0, Kd=0.001`

---

## Stage 3 — Tune the Speed Loop (DCMotorTacho)

Only needed if using `DCMotorTacho`.

The inner (speed) PID output unit is **encoder counts**, fed as a moving setpoint into the outer position loop. Tune it after the position loop is stable.

### 3a — Set speedInterval

The default is 50 ms. The interval must be long enough to accumulate a measurable count change at low speeds. For slow, high-reduction motors, 100–200 ms may be needed.

### 3b — Tune Inner PID Gains

Use `examples/tacho_config/01_tune_speed_pid/`. Plot `getMeasuredSpeedRPM()` vs `getDesiredSpeedRPM()`.

**Procedure:**

| Step | Action                                                                                           |
| ---- | ------------------------------------------------------------------------------------------------ |
| 1    | Set Ki = 0, Kd = 0. Increase Kp until measured speed tracks commanded speed without oscillation. |
| 2    | Add Ki to eliminate steady-state RPM error.                                                      |
| 3    | Kd is rarely needed for the speed loop; leave at 0 unless there is ringing.                      |

**Typical starting point:** `Kp=0.5, Ki=0.1, Kd=0.0`

---

## Quick Reference — Default Values

| Setting           | Default     | Setter                         |
| ----------------- | ----------- | ------------------------------ |
| PWM skip          | 50          | `setPWMSkip(value)`            |
| Max PWM           | 255         | `setMaxPWM(value)`             |
| Position accuracy | 30 cts      | `setAccuracy(value)`           |
| Outer Kp/Ki/Kd    | 0.1/0.2/0.1 | `servo.setPIDTunings(...)`     |
| PID sample time   | 50 ms       | `servo.myPID->SetSampleTime()` |
| Speed interval    | 50 ms       | `DCMotorTacho` constructor     |
| Inner Kp/Ki/Kd    | 0.1/0.2/0.1 | `tacho.setPIDTunings(...)`     |

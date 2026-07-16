# API Reference

## DCMotorServo

### Constructor

```cpp
DCMotorServo(MotorWriteFunc mWrite, MotorBrakeFunc mBrake,
             EncoderReadFunc eRead, EncoderWriteFunc eWrite)
```

Provide four function pointers that adapt your motor driver and encoder library to the control interface. See [getting-started.md](getting-started.md) for the required signatures.

**Defaults set in the constructor:**

| Parameter         | Default       | Description                        |
| ----------------- | ------------- | ---------------------------------- |
| PWM skip          | 50            | Minimum PWM to overcome stiction   |
| Position accuracy | 30            | Acceptable error in encoder counts |
| Max PWM           | 255           | Upper PWM clamp                    |
| PID (Kp, Ki, Kd)  | 0.1, 0.2, 0.1 | Starting tuning values             |
| PID sample time   | 50 ms         |                                    |

---

### Public Member

```cpp
PID *myPID
```

Direct access to the underlying [Arduino PID library](https://github.com/br3ttb/Arduino-PID-Library) object for advanced configuration (e.g. `myPID->SetSampleTime()`).

---

### Motion

#### `run()`

```cpp
void run()
```

Reads the encoder, computes PID output, and sends the result to the motor driver. **Call every loop iteration.** With the extrema features active, `run()` first services the special modes: while homing it drives the homing move instead of the PID (encoder input is not tracked into the PID); while a stall fault is latched it does nothing (motor stays braked); while the target lies beyond a triggered endstop it holds the motor braked and pulls the target to the held position.

#### `stop()`

```cpp
void stop()
```

Engages the brake and suspends the PID controller. Also cancels an in-progress homing move — the target is synced to the current position so the motor holds where it stopped rather than chasing the pre-homing setpoint.

#### `moveTo(position)`

```cpp
void moveTo(long position)
```

Sets an absolute target position in encoder counts.

#### `move(delta)`

```cpp
void move(long delta)
```

Adds `delta` encoder counts to the current target (relative move).

---

### Status

#### `finished()`

```cpp
bool finished()
```

Returns `true` when the encoder is within the accuracy window of the target and the PWM output is zero. Always `false` while a homing move is in progress. A target blocked by a triggered endstop is pulled to the held position, so `finished()` reports `true` once the motor has gone as far as it can.

#### `getActualPosition()`

```cpp
long getActualPosition()
```

Returns the current encoder count.

#### `getRequestedPosition()`

```cpp
long getRequestedPosition()
```

Returns the current PID setpoint (target count).

---

### Configuration

#### `setPIDTunings(Kp, Ki, Kd)`

```cpp
void setPIDTunings(double Kp, double Ki, double Kd)
```

Sets the PID gains. See [tuning.md](tuning.md).

#### `setPWMSkip(range)`

```cpp
bool setPWMSkip(uint8_t range)
```

Sets the minimum PWM added to the PID output to overcome static friction. Determined experimentally. Returns `false` if `range >= 255`.

#### `setMaxPWM(maxPWM)`

```cpp
void setMaxPWM(uint8_t maxPWM)
```

Clamps the maximum PWM output. Also updates PID output limits accordingly.

#### `setAccuracy(range)`

```cpp
void setAccuracy(unsigned int range)
```

Sets the position dead-band in encoder counts. The motor is considered finished when error is below this value.

#### `setCurrentPosition(position)`

```cpp
void setCurrentPosition(long position)
```

Resets the encoder count to `position` and updates the PID input. Useful for zeroing after homing.

---

### Extrema sensing

Three tiers, usable independently or together: software travel limits, physical
endstops, and encoder-based stall detection. A non-blocking homing move ties
them together by zeroing the encoder at a detected extreme.

#### `setTravelLimits(min, max)`

```cpp
void setTravelLimits(long min_position, long max_position)
```

Sets software extrema in encoder counts. Targets passed to `move()`/`moveTo()` are clamped into `[min, max]` (arguments are swapped if given out of order). The current setpoint is clamped immediately.

#### `clearTravelLimits()`

```cpp
void clearTravelLimits()
```

Removes the software travel limits.

#### `attachEndstops(minStop, maxStop)`

```cpp
void attachEndstops(EndstopReadFunc minStop, EndstopReadFunc maxStop)
```

Attaches physical endstop sensors (limit switch, hall effect, ...). Each is a `bool ()` function returning `true` when triggered; pass `nullptr` for a side with no sensor. While the target position lies beyond a triggered endstop, `run()` holds the motor braked and pulls the target to the held position — nothing (including `DCMotorTacho`'s speed loop) can wind the setpoint past the stop, and a later switch release cannot cause an uncommanded move. Motion away from the endstop remains allowed; command a new target to move off the stop. The hold is judged on the direction of the position error, so it is stable — the motor does not chatter against the switch.

#### `enableStallDetection(timeout_ms, min_counts)`

```cpp
void enableStallDetection(unsigned long timeout_ms, long min_counts = 4)
```

Enables encoder-based stall detection: if the motor is being driven but the encoder advances fewer than `min_counts` counts within `timeout_ms`, the motor is braked and a stall fault is latched. During homing, a stall instead marks the extreme (sensorless homing). Choose `timeout_ms` longer than your mechanism's worst-case time to produce `min_counts` at minimum PWM.

#### `disableStallDetection()`

```cpp
void disableStallDetection()
```

Disables stall detection.

#### `isStalled()` / `clearStall()`

```cpp
bool isStalled()
void clearStall()
```

`isStalled()` reports a latched stall fault; while latched, `run()` keeps the motor braked. `clearStall()` clears the fault and resumes normal operation — note the target position is unchanged, so the motor will drive toward it again; re-target first if the cause of the stall is still present.

#### `startHoming(direction, pwm, max_travel)`

```cpp
bool startHoming(int8_t direction, uint8_t pwm, long max_travel = 0)
```

Starts a non-blocking homing move: drives at a fixed `pwm` (clamped to `[pwm_skip, maxPWM]`) toward an extreme — negative `direction` for the minimum side, positive for the maximum side — until the matching endstop triggers or a stall is detected, whichever is attached/enabled. Software travel limits are bypassed during the move, and any latched stall fault is cleared (a stall during homing marks the extreme, not a fault). On success the motor brakes, the encoder is zeroed, and the setpoint is set to the (limit-clamped) zero. `max_travel` is a failsafe bound in encoder counts (e.g. against a dead endstop switch): exceeding it aborts the move *unhomed*, holding the current position; `0` means unbounded. Returns `false` if `direction` is `0` or if neither the matching endstop nor stall detection is available to detect the extreme. Progressed by `run()`; cancel with `stop()`.

#### `isHoming()` / `isHomed()`

```cpp
bool isHoming()
bool isHomed()
```

`isHoming()` returns `true` while a homing move is in progress; poll it after `startHoming()` to detect the end of the move. `isHomed()` reports whether the last homing move actually *succeeded* (encoder zeroed at a detected extreme) — it stays `false` when homing was cancelled by `stop()`, aborted by `max_travel`, or lost its termination conditions mid-move. `isHoming() == false` alone does not imply success; check `isHomed()`.

---

### Diagnostics

#### `getDebugInfo()`

```cpp
String getDebugInfo()
```

Returns a multi-line string with encoder position, PID setpoint/input/output, PWM skip, accuracy, travel limits (when set), stall/homing state, and PID gains.

#### `getSerialPlotter(id)`

```cpp
String getSerialPlotter(int id = -1)
```

Returns a comma-separated `Setpoint:x, Input:x, Output:x` string formatted for the Arduino Serial Plotter. Pass an integer `id` when plotting multiple motors to differentiate traces.

---

## DCMotorTacho

Wraps a `DCMotorServo` with a cascaded speed (inner) PID loop. The inner loop measures RPM and produces a moving encoder setpoint; the outer position loop in `DCMotorServo` tracks it.

### Constructor

```cpp
DCMotorTacho(DCMotorServo *servo, double cpr, unsigned long speedInterval = 50)
```

| Parameter       | Description                                                                            |
| --------------- | -------------------------------------------------------------------------------------- |
| `servo`         | Pointer to an existing, configured `DCMotorServo` object                               |
| `cpr`           | Encoder counts per revolution (after gearbox + quadrature). See [tuning.md](tuning.md) |
| `speedInterval` | Inner loop update interval in ms (default 50 ms)                                       |

---

### Methods

#### `run()`

```cpp
void run()
```

Runs both the inner speed loop and the outer position loop. **Call every loop iteration.** While the wrapped servo is homing or stall-latched, the speed loop is held (its PID suspended and its counters kept in sync) so it cannot wind the position setpoint against a frozen encoder, and the zero-speed branch cannot silently cancel a homing move (an explicit `stop()` still cancels, as documented); normal speed control resumes automatically afterwards.

#### `stop()`

```cpp
void stop()
```

Brakes the motor and disables the inner PID.

#### `setSpeedRPM(rpm)`

```cpp
void setSpeedRPM(double rpm)
```

Sets the desired shaft speed in RPM. Negative values reverse direction. `0` engages the brake.

#### `getDesiredSpeedRPM()`

```cpp
double getDesiredSpeedRPM() const
```

Returns the commanded RPM.

#### `getMeasuredSpeedRPM()`

```cpp
double getMeasuredSpeedRPM() const
```

Returns the most recently calculated actual RPM.

#### `setPIDTunings(Kp, Ki, Kd)`

```cpp
void setPIDTunings(double Kp, double Ki, double Kd)
```

Sets the inner (speed) PID gains. The outer (position) gains are set on the underlying `DCMotorServo`.

#### `getServo()`

```cpp
DCMotorServo *getServo() const
```

Returns the underlying `DCMotorServo` pointer for direct access.

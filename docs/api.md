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

Reads the encoder, computes PID output, and sends the result to the motor driver. **Call every loop iteration.**

#### `stop()`

```cpp
void stop()
```

Engages the brake and suspends the PID controller.

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

Returns `true` when the encoder is within the accuracy window of the target and the PWM output is zero.

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

### Diagnostics

#### `getDebugInfo()`

```cpp
String getDebugInfo()
```

Returns a multi-line string with encoder position, PID setpoint/input/output, PWM skip, accuracy, and PID gains.

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

Runs both the inner speed loop and the outer position loop. **Call every loop iteration.**

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

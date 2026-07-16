# Usage Demos

Ready-to-run sketches showing DCMotorServo/DCMotorTacho with real driver hardware:

| Demo | Driver | Shows |
| --- | --- | --- |
| `l298n_dual` | L298N | Two independent position servos |
| `l298n_speed` | L298N | Speed control via DCMotorTacho |
| `l298n_tune` | L298N | Interactive PID tuning over serial |
| `l298n_homing` | L298N | Extrema sensing: endstop + sensorless homing, travel limits, stall recovery |
| `lmd18200_dual` | LMD18200 | Two independent position servos |
| `lmd18200_speed` | LMD18200 | Speed control via DCMotorTacho |

The demos depend on the matching driver library — [L298N](https://github.com/CameronBrooks11/L298N_Arduino) or [LMD18200](https://github.com/CameronBrooks11/lmd18200) — in addition to DCMotorServo's own dependencies. See [docs/getting-started.md](../../docs/getting-started.md#example-driver-libraries).

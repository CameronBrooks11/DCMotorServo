# Speed PID Tuning Procedure

Interactive tuning of the inner (speed) PID. Upload with your calibrated defines (see the parent README), open the **Serial Plotter** at 115200 baud. The stream shows the outer loop (`Setpoint, Input, Output`) plus `MeasuredSpeedRPM` and `DesiredSpeedRPM` — you are shaping `MeasuredSpeedRPM` onto `DesiredSpeedRPM`.

## Commands

| Command | Effect |
| --- | --- |
| `SPEED=<rpm>` | Set the desired speed (negative reverses; `SPEED=0` brakes) |
| `SPEEDKP=` / `SPEEDKI=` / `SPEEDKD=` | Inner (speed) PID gains |
| `KP=` / `KI=` / `KD=` | Outer (position) PID gains — normally already tuned in servo_config |

## Procedure

1. Command a mid-range speed for your application, e.g. `SPEED=60`.
2. Tune `SPEEDKP` up until `MeasuredSpeedRPM` tracks briskly with slight oscillation, then back off.
3. Add `SPEEDKI` until steady-state RPM error disappears; too much shows as slow RPM hunting.
4. `SPEEDKD` is rarely needed — the RPM measurement is already differentiated from counts and is noisy; prefer a longer `SPEED_INTERVAL` over derivative action if the plot is ragged.
5. Verify across the speed range you care about (low speeds stress the measurement resolution — see the parent README's interval rule of thumb) and in both directions, and check `SPEED=0` engages the brake cleanly.
6. Record the results in the parent README's sample-results table.

## What good looks like

`MeasuredSpeedRPM` settling on `DesiredSpeedRPM` within a few intervals, no sustained hunting, and the outer-loop `Input` ramping smoothly (the inner loop moves the position setpoint; jagged outer input means the inner output is slamming).

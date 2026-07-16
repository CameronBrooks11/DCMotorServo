# Servo PID Tuning Procedure

Interactive step-response tuning of the position PID. Upload the sketch, open the **Serial Plotter** at 115200 baud, and drive the tuning with serial commands. Set the driver `#define` (`USE_LMD18500` / `USE_L298N`) and your PWM-skip value from step 01 and accuracy from step 02 before uploading.

## Commands

| Command | Effect |
| --- | --- |
| `MOVE=<counts>` | Relative position step (e.g. `MOVE=5000`) |
| `KP=<val>` / `KI=<val>` / `KD=<val>` | Set a PID gain live |
| `MAXPWM=<val>` | Cap the PWM output |

The plotter streams `Setpoint`, `Input`, `Output` — the step response you are shaping.

## Procedure

1. Start from the defaults (`KP=0.1 KI=0.15 KD=0.05`) and command a mid-size step, e.g. `MOVE=5000`.
2. **Tune KP first** (`KI=0 KD=0` optionally): raise until the response tracks the setpoint briskly and just begins to overshoot or oscillate, then back off ~20%.
3. **Add KI** to remove the steady-state gap between Input and Setpoint. Too much shows as slow oscillation / windup after the step.
4. **Add KD** only if overshoot persists; too much amplifies encoder noise (jittery Output at rest).
5. Confirm with steps in both directions and at small (`MOVE=500`) and large (`MOVE=20000`) magnitudes — a tune that only works at one scale usually means PWM skip (step 01) is off.
6. Record the final gains in the top-level `servo_config/README.md` sample-results table and hardcode them in your sketch via `setPIDTunings()`.

## What good looks like

Fast rise, at most one small overshoot, Input settling flat on Setpoint with Output dropping to zero (the accuracy dead-band from step 02 disengages the PID at target).

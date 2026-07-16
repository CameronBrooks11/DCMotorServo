# Tacho Config

Walks through configuring the cascaded speed loop (`DCMotorTacho`) for your motor. **Prerequisite:** complete `servo_config/` first — the speed loop drives the position loop, so it inherits your PWM skip, accuracy, position PID gains, and calibrated CPR.

## Procedure

0. Carry over from servo_config: position PID gains (step 03) and the empirical fudge factor / CPR (step 04). Carry your filled-in `baseSpecs.h` forward from servo_config (it holds `PPR`, `GEAR_RATIO`, `EMPIRICAL_FUDGE_FACTOR`, and the rest) — an uncalibrated CPR makes every RPM reading proportionally wrong.
1. **speedInterval**: the inner-loop update period (`SPEED_INTERVAL`, default 20 ms). Shorter reacts faster but measures fewer counts per window, so RPM quantizes noisily at low speeds — as a rule of thumb keep `CPR × RPM_min / 60000 × interval ≳ 4` counts. Raise it (e.g. 50 ms) for low-CPR encoders or low-speed applications.
2. **Speed PID tuning**: the `01_tune_speed_pid` sketch — see its README.

## Sample Results

- 26PG-3429-19-EN DC motor (PPR 12, gear 19:1, fudge 0.875)

| Parameter | L298N | LMD18200 |
| --- | --- | --- |
| SPEED_INTERVAL | | |
| Speed Kp, Ki, Kd | | |

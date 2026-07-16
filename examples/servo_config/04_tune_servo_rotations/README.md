# Rotational Tuning Procedure (CPR Empirical Fudge Factor)

Calibrates the empirical fudge factor so that commanded rotations match physical rotations exactly. Requires the PID gains from step 03. Set `PPR` and `GEAR_RATIO` from your motor's datasheet in the sketch defines; CPR is computed as `PPR × GEAR_RATIO × 4 × fudge`.

## Commands

| Command | Effect |
| --- | --- |
| `MOVEROTATE=<revs>` | Move a whole number of output-shaft revolutions at the current CPR |
| `FUDGE=<val>` | Set the fudge factor (recomputes CPR live) |
| `KP=` / `KI=` / `KD=` / `MAXPWM=` | Same as step 03, if the tune needs touching up |

## Procedure

1. Mark the output shaft (tape flag or marker line against a fixed reference).
2. With `FUDGE=1.0`, command `MOVEROTATE=1` and observe where the mark lands.
3. If it over- or under-rotates, correct proportionally: turned 370° instead of 360° → `FUDGE=0.973` (= 360/370).
4. Verify over a longer move where the error accumulates visibly: `MOVEROTATE=10` should land the mark exactly back on the reference. Refine with small fudge increments.
5. Test both directions — a direction-dependent discrepancy indicates mechanical backlash, not CPR error; calibrate on the average.
6. Record the final fudge factor in the sample-results table and use the resulting CPR everywhere (it is the `cpr` argument to `DCMotorTacho`).

## Notes

Datasheet PPR values are sometimes nominal; gearbox ratios like "19:1" can really be 18.75:1 — that is exactly what the fudge factor absorbs. A fudge far from 1.0 (outside ~0.9–1.1) usually means `PPR` or `GEAR_RATIO` is wrong, not the mechanism.

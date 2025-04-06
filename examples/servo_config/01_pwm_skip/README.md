# PWM Skip Tuning Procedure

Upload the script and note the final output value. Monitor the temperature of your driver and remove power immediately if it gets excessively hot.

## Algorithm

### Initialization

1. **Constants:**

   - **INIT_PWM_RAMP_INTERVAL:** The initial interval (in milliseconds) between PWM increments (e.g., 400 ms).
   - **MOVE_STEPS_SENSITIVITY:** The minimum encoder steps difference considered as intentional movement (e.g., 5 steps).
   - **MAX_PWM:** The maximum PWM value (typically 255).

2. **Variables:**
   - **pwmRampInterval:** Initialized to INIT_PWM_RAMP_INTERVAL.
   - **pwmStart:** The starting PWM value for the current iteration, initially set to 0.
   - **currentPWM:** The PWM value currently applied; initially 0.
   - **pwmEstimate:** The latest PWM estimate from encoder detection; initially set to MAX_PWM.
   - **pwmEstimate0:** The PWM value recorded when significant movement is first detected.

### Setup

1. Zero the encoder.
2. Initialize `pwmRampInterval` to **INIT_PWM_RAMP_INTERVAL**.
3. Set `pwmStart` and `currentPWM` to 0.
4. Set `pwmEstimate` to **MAX_PWM**.
5. Apply a brief brake (with a short delay) to let the motor settle before starting the tuning.

### Loop

1. **Encoder Check (every 50 ms):**
   - Read the encoder count.
   - **If** the absolute encoder count is greater than or equal to **MOVE_STEPS_SENSITIVITY**:
     - Record the current PWM value as `pwmEstimate0`.
     - Set `pwmEstimate` equal to `pwmEstimate0`.
     - Update `pwmStart` to halfway between the previous `pwmStart` and the new estimate:
       - `pwmStart = pwmEstimate0 - ((pwmEstimate0 - pwmStart) / 2)`
     - Double the ramp interval:
       - `pwmRampInterval = pwmRampInterval * 2`
     - Print the current parameters (PWM_Start, Current_PWM, PWM_Estimate, Ramp_Interval, Encoder reading).
     - Apply a brake and pause briefly to allow the motor to settle.
     - Reset the encoder.
     - Set `currentPWM` to the new `pwmStart`.
     - **Termination Check:** If `pwmEstimate` equals `pwmStart`, the tuning is complete.
2. **PWM Ramp Update (every pwmRampInterval ms):**

   - **If** no significant encoder movement is detected:
     - Increment `currentPWM` by 1 (ensuring it does not exceed **MAX_PWM**).
     - Apply the new PWM.
     - Print the current parameters for monitoring.

3. **Repeat** steps 1–2 until tuning completes.

4. **Output:**  
   When tuning completes, the final values of `pwmStart` and `pwmEstimate` are reported.

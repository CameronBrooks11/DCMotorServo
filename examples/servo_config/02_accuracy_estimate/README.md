# Accuracy Estimate Tuning Procedure

## Algorithm

### Initialization

1. Define constants:
   - **INIT_ACCURACY**: The initial accuracy threshold.
   - **MAX_PWM**: The motor drive signal when moving forward/backward.
   - **targetSteps**: The encoder count target to reach before attempting to stabilize.

### Setup

1. Initialize serial communication and motor driver.
2. Zero the encoder.
3. Set the current `accuracy` threshold to **INIT_ACCURACY**.

### Loop

1. For each accuracy level, attempt **TRIALS_PER_ACCURACY** consecutive stabilization trials:

   - a. Reset the encoder count.
   - b. Run a bang-bang control loop for a fixed stabilization period:
     - If below `targetSteps - accuracy`, drive forward at `MAX_PWM`.
     - If above `targetSteps + accuracy`, drive backward at `-MAX_PWM`.
     - If within range, brake the motor.
     - Print status to serial every 100 ms.
   - c. After the period ends, measure final error.
   - d. If final error is within the threshold:
     - Count as a successful trial.
     - If all required trials succeed, refine the `accuracy` threshold by decrementing it.
   - e. If the trial fails:
     - Reset the success counter.
     - If **no** trial succeeded at all:
       - Finalize the current accuracy as the best achievable.
       - Report and halt.

2. Repeat the loop for finer accuracy thresholds, until no further refinement is possible (i.e., `currentAccuracy` cannot decrease).

3. Continuously print current parameters and progress to Serial.

### Termination

- If all trials at a given accuracy level fail:

  - Finalize and report the current accuracy as the final achievable value.
  - Halt further execution.

- If the `accuracy` threshold reaches its minimum (e.g., 1) and no further improvement is possible:
  - Report and halt.

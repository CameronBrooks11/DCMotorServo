# Servo Config

This directory contains a series of subdirectories to walk the user through the hardware setup and firmware configuration for operating a brushed (BDC) DC motor as a rotational positioning actuator similar to a servo except not angle-limited.

## Procedure

0. Ensure you have connected your chosen motor and driver and noted the connections to your MCU.
1. PWM Skip: the 01_pwm_skip directory provides a script to determine the PWM skip value for your specific motor.
2. Accuracy Estimate: the 02_accuracy_estimate provides a script to estimate an appripriate accuracy value.
3. PID Tuning: Upload the script and open the serial plotter to view outputs adn open serial plotter and send commands (i.e. KP=0.5, MOVE=5000, etc) to tune the PID constants and apply position steps.
4. Rotational Tuning: Upload the script and open the serial monitor and send commands (i.e. FUDGE=0.95, MOVEROTATION=5, etc) to tune the the empirical fudge factor by applying incrementatal changes to fudge while testing the resulting rotation.

## Sample Results

- 26PG-3429-19-EN DC motor
- L298N driver (breakout module) and LMD18500 on custom PCB (Slice_DCMT)

### PWM Skip

- L298N: 35/255
- LMD18500: 15/255

### Accuracy

- L298N: 4
  - start at x2 = 8
- LMD18500:

### PID Tuning

- L298N: K=x, I=y, D=z
- LMD18500:

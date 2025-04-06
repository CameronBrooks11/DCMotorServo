# Servo Config

This directory contains a series of subdirectories to walk the user through the hardware setup and firmware configuration for operating a brushed (BDC) DC motor as a rotational positioning actuator similar to a servo except not angle-limited.

## Procedure

0. Ensure you have connected your chosen motor and driver and noted the connections to your MCU.
1. PWM Skip: the 01_pwm_skip directory provides a script to determine the PWM skip value for your specific motor.
2. Accuracy Estimate: the 02_accuracy_estimate provides a script to estimate an appripriate accuracy value.

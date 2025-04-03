# Servo Config

This directory contains a series of subdirectories to walk the user through the hardware setup and firmware configuration for operating a brushed (BDC) DC motor as a rotational positioning actuator similar to a servo except not angle-limited.

## Procedure

0. [Setup](#setup): the top level of this directory is dedicated to the hardware connections and testing of your motor driver for the rest of the procedure.
1. PWM Skip: the 01_pwm_skip directory provides a script to determine the PWM skip value for your specific motor.
2. Accuracy Estimate: the 02_accuracy_estimate provides a script to estimate an appripriate accuracy value.

## Setup

Start by filling out the specs of your motor and its encoder in baseSpecs.h, to do this you will need the following information.

table here

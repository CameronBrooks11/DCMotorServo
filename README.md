# DCMotorServo

An Arduino Library for controlling DC motors with rotary encoders. This library is an updated version and continuation of [julester23 / DCMotorServo](https://github.com/julester23/DCMotorServo) uses PID and Encoder feedback, inspired by the [adafruit / AccelStepper](https://github.com/adafruit/AccelStepper) library.

- Encoder Library, for measuring quadrature encoded signals from the Arduino library manager and also on [pjrc.com/teensy/td_libs_Encoder.html](http://www.pjrc.com/teensy/td_libs_Encoder.html)
- PID Library, for using encoder feedback to control the motor from the Arduino library manager and also on [github.com/br3ttb/Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)

## Circuit
I used a 754410 quad half-H controller (a pin-compatible L293D). I'm sure it would be cheaper to make out of other components, but I've never done transistor matching, and I'm afraid of burning things.

### Example circuit connections
| L293D or 754410 pins | Device |        |
|----------------------|--------|--------|
| 1, 9                 | arduino| pin_pwm_output |
| 2, 15                | arduino| pin_dir |
| 7, 10                | arduino| pin_dir |
| 4, 5, 12, 13         | power  | GND    |
| 16                   | power  | 5V     |
| 8                    | power  | 12V    |
| 3, 14                | motor  | motor pin 1 |
| 6, 11                | motor  | motor pin 2 |

## Hardware
- Example 1: Move_1inch
  - [Metal Gearmotor 37Dx57L mm with 64 CPR Encoder from Pololu](http://www.pololu.com/catalog/product/1447)
  - Arduino
  - [MC33926 Motor Driver Carrier from Pololu](http://www.pololu.com/product/1212)
- Example 2: XXX
  - [20D Planetary Gearmotor w/ Encoder - 12V 515RPM by E-S Motor via RobotShop](https://ca.robotshop.com/products/20d-planetary-gearmotor-w-encoder-12v-515rpm)
  - Arduino Nano
  - [298N Motor Drive Controller Board Module](https://ca.robotshop.com/products/20d-planetary-gearmotor-w-encoder-12v-515rpm)

## Pins
Pinout for motor control uses 3 pins for output. It is somewhat wasteful, but had more flexibility. Two pins for direction control, and one for motor power (assuming PWM).
Be sure to pick a PWM capable pin for pin_pwm_output.

The two input pins are for the encoder feedback.

Two directional pins allow for setting a motor brake by shorting the terminals of the motor together (set both directions HIGH, and preferably turn off the PWM)

## TODO
- implement brake feature for 3-pin mode
- 2-pin constructor
- implement friendlier tuning method for PID
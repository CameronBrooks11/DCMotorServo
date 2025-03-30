/**
 * @file DCMotorServo.h
 * @brief Control library for DC motors with PID and encoder feedback.
 *
 * This library provides an interface for controlling DC motors using PID feedback from an encoder. It is designed
 * for use with motors equipped with quadrature encoders and supports various features including setting motor speed
 * and position, stopping, and configuring PID parameters.
 *
 * @note This library is modeled a bit after the AccelStepper library and uses the Encoder library for measuring quadrature encoded signals
 * and the PID library for control.
 *
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 */

#ifndef DCMotorServo_h
#define DCMotorServo_h

#include <Encoder.h>

#include <PID_v1.h>

/**
 * @class DCMotorServo
 * @brief A class to control DC motors with encoder feedback using PID.
 *
 * Provides methods to control a DC motor's position or speed using a PID controller and feedback from an encoder.
 * It is compatible with motors controlled via a H-bridge like the 754410.
 */
class DCMotorServo
{
public:
  /**
   * Constructor for the DCMotorServo class.
   * @param pin_dir_1 First direction control pin.
   * @param pin_dir_2 Second direction control pin.
   * @param pin_pwm_output PWM output pin for motor speed control.
   * @param pin_encode1 First encoder input pin.
   * @param pin_encode2 Second encoder input pin.
   */
  DCMotorServo(uint8_t pin_dir_1 = 4, uint8_t pin_dir_2 = 5, uint8_t pin_pwm_output = 6, uint8_t pin_encode1 = 2, uint8_t pin_encode2 = 3);

  PID *myPID; ///< Pointer to PID object for motor control.

  /**
   * Initiates motor movement based on PID calculations.
   */
  void run();

  /**
   * Stops the motor by disabling its PWM signal.
   */
  void stop();

  /**
   * Moves the motor to a new relative position.
   * @param new_rela_position The new position relative to the current position.
   */
  void move(int new_rela_position);

  /**
   * Moves the motor to a specific absolute position.
   * @param new_position The target position for the motor.
   */
  void moveTo(int new_position);

  /**
   * Retrieves the current target position of the motor.
   * @return The motor's target position.
   */
  int getRequestedPosition();

  /**
   * Retrieves the actual current position of the motor.
   * @return The current position as reported by the encoder.
   */
  int getActualPosition();

  /**
   * Checks if the motor has reached its target position within a defined accuracy.
   * @return True if the motor is at the target position, false otherwise.
   */
  bool finished();

  /**
   * Sets the PWM skip range.
   * @param range The PWM value below which the motor should not move.
   * @return True if the range is set successfully, false otherwise.
   */
  bool setPWMSkip(uint8_t);

  /**
   * Sets the maximum allowable PWM value to control the motor speed.
   * @param maxPWM The maximum PWM value, which caps the speed.
   */
  void setMaxPWM(uint8_t maxPWM);

  /**
   * Sets the PID tuning parameters.
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   */
  void SetPIDTunings(double Kp, double Ki, double Kd);

  /**
   * Sets the accuracy for the motor position.
   * @param range The highest tolerable inaccuracy in encoder counts.
   */
  void setAccuracy(unsigned int);

  /**
   * Sets the current position of the motor.
   * @param new_position The new position of the motor.
   */
  void setCurrentPosition(int);

  /**
   * Enables developer/debug mode, providing status information about the encoder and current PID parameters.
   * @return A string containing relevant information for debugging and tuning the motor controller.
   */
  String getDebugInfo();

  /**
   * Generates a string formatted for the Serial Plotter in Arduino IDE, showing
   * current PID values for real-time tuning and debugging.
   * @return A string of comma-separated PID values: setpoint, input, and output.
   */
  String getSerialPlotter();

private:
  uint8_t _pin_PWM_output, _pin_dir_1, _pin_dir_2; ///< Pin assignments.
  double _PID_setpoint, _PID_input, _PID_output;   ///< PID control variables.
  uint8_t _PWM_output;                             ///< The PWM value for motor output.

  Encoder *_position;         ///< Pointer to the Encoder object.
  uint8_t _pwm_skip;          ///< Range of PWM to skip for low power movement.
  uint8_t _maxPWM; ///< The maximum PWM value to prevent the motor from going too fast.
  uint8_t _position_accuracy; ///< Tolerance for position accuracy.

  /**
   * Determines the direction of motor rotation based on PID output.
   */
  void _pick_direction();
};

#endif // DCMotorServo_h

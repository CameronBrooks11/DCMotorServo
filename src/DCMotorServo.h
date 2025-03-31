// DCMotorServo.h
#ifndef DCMotorServo_h
#define DCMotorServo_h

#include <PID_v1.h>
#include <Arduino.h>

// Define function pointer types for hardware abstraction:
typedef void (*MotorWriteFunc)(int16_t speed);
typedef void (*MotorBrakeFunc)();
typedef int (*EncoderReadFunc)();
typedef void (*EncoderWriteFunc)(int newPosition);

/**
 * @class DCMotorServo
 * @brief A class to control DC motors with encoder feedback using PID.
 *
 * Provides methods to control a DC motor's position or speed using a PID controller and feedback from an encoder.
 * This version is driver-agnostic: hardware-specific operations are provided via function pointers.
 */
class DCMotorServo
{
public:
  /**
   * Constructor for the DCMotorServo class.
   * @param mWrite Function pointer for writing a motor command (signed speed value).
   * @param mBrake Function pointer for braking the motor.
   * @param eRead  Function pointer for reading the encoder position.
   * @param eWrite Function pointer for writing/resetting the encoder position.
   */
  DCMotorServo(MotorWriteFunc mWrite, MotorBrakeFunc mBrake, EncoderReadFunc eRead, EncoderWriteFunc eWrite);

  PID *myPID; ///< Pointer to PID object for motor control.

  /**
   * Runs the PID controller and sends the computed output to the motor driver.
   */
  void run();

  /**
   * Stops the motor by invoking the brake function.
   */
  void stop();

  /**
   * Moves the motor to a new relative position.
   * @param new_rela_position The relative position to add to the current target.
   */
  void move(int new_rela_position);

  /**
   * Moves the motor to an absolute position.
   * @param new_position The target position.
   */
  void moveTo(int new_position);

  /**
   * Retrieves the current target position.
   * @return The motor's target position.
   */
  int getRequestedPosition();

  /**
   * Retrieves the actual current position from the encoder.
   * @return The current encoder reading.
   */
  int getActualPosition();

  /**
   * Checks if the motor has reached its target position within defined accuracy.
   * @return True if the motor is at the target position, false otherwise.
   */
  bool finished();

  /**
   * Sets the PWM skip value, which defines the minimal PWM level to overcome low-power stalling.
   * @param range Minimum PWM value.
   * @return True if set successfully, false otherwise.
   */
  bool setPWMSkip(uint8_t range);

  /**
   * Sets the maximum PWM value allowed.
   * @param maxPWM Maximum PWM value.
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
   * Sets the tolerance for position accuracy.
   * @param range Maximum allowable error.
   */
  void setAccuracy(unsigned int range);

  /**
   * Sets the current encoder position.
   * @param new_position The new encoder position.
   */
  void setCurrentPosition(int new_position);

  /**
   * Provides debugging information.
   * @return A string with current status and PID parameters.
   */
  String getDebugInfo();

  /**
   * Generates a string formatted for Serial Plotter.
   * @param id Optional ID for the motor (default is -1 which means no ID).
   *        This is useful for distinguishing between multiple motors in the plotter.
   * @return A comma-separated string of PID setpoint, input, and output.
   */
  String getSerialPlotter(int id = -1);

private:
  // PID control variables:
  double _PID_setpoint, _PID_input, _PID_output;
  uint8_t _PWM_output;        // Computed PWM value
  uint8_t _pwm_skip;          // Minimum PWM to overcome low-power stall
  uint8_t _maxPWM;            // Maximum allowable PWM value
  uint8_t _position_accuracy; // Tolerance for position accuracy

  // Function pointers for hardware abstraction:
  MotorWriteFunc motorWrite;
  MotorBrakeFunc motorBrake;
  EncoderReadFunc encoderRead;
  EncoderWriteFunc encoderWrite;
};

#endif // DCMotorServo_h

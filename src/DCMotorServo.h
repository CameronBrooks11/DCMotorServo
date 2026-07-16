// DCMotorServo.h
#ifndef DCMotorServo_h
#define DCMotorServo_h

#include <PID_v1.h>
#include <Arduino.h>

// Define function pointer types for hardware abstraction:
typedef void (*MotorWriteFunc)(int16_t speed);
typedef void (*MotorBrakeFunc)();
typedef long (*EncoderReadFunc)();
typedef void (*EncoderWriteFunc)(long newPosition);
typedef bool (*EndstopReadFunc)(); // true = endstop triggered

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
  void move(long new_rela_position);

  /**
   * Moves the motor to an absolute position.
   * @param new_position The target position.
   */
  void moveTo(long new_position);

  /**
   * Retrieves the current target position.
   * @return The motor's target position.
   */
  long getRequestedPosition();

  /**
   * Retrieves the actual current position from the encoder.
   * @return The current encoder reading.
   */
  long getActualPosition();

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
  void setPIDTunings(double Kp, double Ki, double Kd);

  /**
   * Sets the tolerance for position accuracy.
   * @param range Maximum allowable error.
   */
  void setAccuracy(unsigned int range);

  /**
   * Sets the current encoder position.
   * @param new_position The new encoder position.
   */
  void setCurrentPosition(long new_position);

  /**
   * Sets software travel limits (extrema in encoder counts).
   * Targets passed to move()/moveTo() are clamped into [min_position, max_position].
   * @param min_position Lowest allowed target position.
   * @param max_position Highest allowed target position.
   */
  void setTravelLimits(long min_position, long max_position);

  /**
   * Removes software travel limits.
   */
  void clearTravelLimits();

  /**
   * Attaches physical endstop sensors (limit switch, hall effect, ...).
   * When a triggered endstop blocks the commanded direction, the motor is braked.
   * @param minStop Function returning true when the minimum-side endstop is triggered (or nullptr).
   * @param maxStop Function returning true when the maximum-side endstop is triggered (or nullptr).
   */
  void attachEndstops(EndstopReadFunc minStop, EndstopReadFunc maxStop);

  /**
   * Enables encoder-based stall detection: if the motor is driven but the encoder
   * advances less than min_counts within timeout_ms, the motor is braked and a
   * stall fault is latched (see isStalled()/clearStall()). During homing a stall
   * is treated as reaching the extreme, not as a fault.
   * @param timeout_ms Window with no movement before declaring a stall.
   * @param min_counts Minimum encoder counts that qualify as movement (default 4).
   */
  void enableStallDetection(unsigned long timeout_ms, long min_counts = 4);

  /**
   * Disables stall detection.
   */
  void disableStallDetection();

  /**
   * Reports whether a stall fault is latched. While latched, run() holds the
   * motor braked until clearStall() is called.
   * @return True if stalled.
   */
  bool isStalled();

  /**
   * Clears a latched stall fault and resumes normal operation.
   */
  void clearStall();

  /**
   * Starts a non-blocking homing move: drives at a fixed PWM toward an extreme
   * until the matching endstop triggers or a stall is detected (whichever is
   * attached/enabled), then brakes and zeroes the encoder there. Progressed by
   * run(); poll isHoming() for completion. Cancelled by stop().
   * @param direction Negative for the minimum-side extreme, positive for maximum-side.
   * @param pwm Drive PWM during homing (clamped to [pwm_skip, maxPWM]).
   * @return False if direction is 0 or no endstop/stall detection can terminate the move.
   */
  bool startHoming(int8_t direction, uint8_t pwm);

  /**
   * Reports whether a homing move is in progress.
   * @return True while homing.
   */
  bool isHoming();

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

  // Software travel limits:
  bool _limits_enabled;
  long _limit_min, _limit_max;

  // Physical endstops (nullptr = not attached):
  EndstopReadFunc endstopMin;
  EndstopReadFunc endstopMax;

  // Stall detection:
  bool _stall_enabled;
  bool _stalled;                  // Latched stall fault
  unsigned long _stall_timeout;   // ms without movement before declaring a stall
  long _stall_min_counts;         // Encoder counts that qualify as movement
  unsigned long _stall_last_time; // Start of the current no-movement window
  long _stall_last_pos;           // Encoder position at window start

  // Homing:
  bool _homing;
  int8_t _homing_dir;
  uint8_t _homing_pwm;

  long clampToLimits(long position); // Apply software travel limits
  void resetStallWindow();           // Restart the no-movement window
  bool stallDetected();              // True if the window expired without movement
  void runHoming();                  // Homing branch of run()
  void haltMotor();                  // Brake and suspend the PID

  // Function pointers for hardware abstraction:
  MotorWriteFunc motorWrite;
  MotorBrakeFunc motorBrake;
  EncoderReadFunc encoderRead;
  EncoderWriteFunc encoderWrite;
};

#endif // DCMotorServo_h

#ifndef DCMotorTacho2_h
#define DCMotorTacho2_h

#include "DCMotorServo.h"
#include <PID_v1.h>
#include <Arduino.h>

/**
 * @class DCMotorTacho2
 * @brief A cascaded speed-control layer that wraps a DCMotorServo object.
 *
 * This class implements a dual PID loop:
 * - The inner (speed) loop compares the desired speed (RPM) to the measured speed (RPM)
 *   and computes an output in encoder counts (delta setpoint).
 * - The outer (position) loop is the existing PID loop in DCMotorServo,
 *   which drives the motor so that the encoder count follows the moving setpoint.
 *
 * The inner PID output is limited by default to ±(CPR×10).
 * If the desired speed is set to 0, the inner PID is disabled and the brake is engaged.
 * Negative desired speeds are handled properly to reverse the motor direction.
 */
class DCMotorTacho2
{
public:
    /**
     * Constructor for DCMotorTacho2.
     * @param servo Pointer to an existing DCMotorServo object.
     * @param cpr Counts per revolution (CPR) for the motor.
     * @param speedInterval Speed update interval in ms (default 50).
     */
    DCMotorTacho2(DCMotorServo *servo, double cpr, unsigned long speedInterval = 50);

    /// Set the desired speed in RPM.
    void setSpeedRPM(double rpm);
    /// Get the desired speed (RPM).
    double getDesiredSpeedRPM() const;
    /// Set inner PID tunings for speed control.
    void setSpeedPIDTunings(double Kp, double Ki, double Kd);
    /// Get the most recently measured speed (RPM).
    double getMeasuredSpeedRPM() const;
    /**
     * Run the cascaded control loops (inner speed loop + outer position loop).
     * If the desired speed is 0, inner PID is disabled and the motor is stopped.
     */
    void run();
    /// Get a pointer to the underlying DCMotorServo.
    DCMotorServo *getServo() const;

private:
    DCMotorServo *_servo;         // Underlying position controller.
    double _desiredSpeedRPM;      // Desired speed (RPM) command.
    double _measuredSpeedRPM;     // Measured speed (RPM).
    double _CPR;                  // Encoder counts per revolution.
    unsigned long _speedInterval; // Speed update interval in ms.

    // Variables for speed measurement:
    unsigned long _lastSpeedUpdate; // Timestamp of last inner loop update.
    long _lastEncoderCount;         // Encoder count at last update.

    // Inner PID variables for speed control:
    double _speedPIDInput;    // Measured speed (RPM)
    double _speedPIDOutput;   // Inner PID output (delta encoder counts)
    double _speedPIDSetpoint; // Desired speed (RPM) – should equal _desiredSpeedRPM.
    PID *_speedPID;           // Inner PID controller.
};

#endif // DCMotorTacho2_h

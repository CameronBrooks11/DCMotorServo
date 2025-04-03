#ifndef DCMotorTacho_h
#define DCMotorTacho_h

#include "DCMotorServo.h"

/**
 * @class DCMotorTacho
 * @brief A subclass of DCMotorServo for speed control using tachometer feedback.
 *
 * This class inherits the PID/position control functionality from DCMotorServo and adds a layer
 * that integrates a desired speed command (in RPM) into a continuously updated position setpoint.
 */
class DCMotorTacho : public DCMotorServo
{
public:
    /**
     * Constructor for DCMotorTacho.
     * @param mWrite   Motor write function pointer.
     * @param mBrake   Motor brake function pointer.
     * @param eRead    Encoder read function pointer.
     * @param eWrite   Encoder write function pointer.
     * @param cpr      Counts per revolution (CPR) for the motor.
     * @param speedInterval  Speed update interval in milliseconds.
     */
    DCMotorTacho(MotorWriteFunc mWrite, MotorBrakeFunc mBrake, EncoderReadFunc eRead, EncoderWriteFunc eWrite,
                 double cpr, unsigned long speedInterval = 50);

    /**
     * Set the desired speed (in RPM).
     */
    void setSpeedRPM(double rpm);

    /**
     * Get the desired speed (in RPM).
     */
    double getDesiredSpeedRPM() const;

    /**
     * Update the moving position setpoint based on the desired speed.
     * This method integrates the speed command over the elapsed time.
     */
    void updateSpeedControl();

    /**
     * Update the measured speed based on encoder counts over a fixed interval.
     */
    void updateMeasuredSpeed();

    /**
     * Overloaded run method that updates speed control (both integration and measured speed)
     * before executing the PID loop.
     */
    void runSpeed();

    /**
     * Get the most recently calculated measured speed (in RPM).
     */
    double getMeasuredSpeedRPM() const;

private:
    double _desiredSpeedRPM;      // Commanded speed (RPM)
    double _CPR;                  // Encoder counts per revolution
    unsigned long _speedInterval; // Interval (ms) for speed updates

    // Variables for setpoint integration (speed control)
    unsigned long _lastSpeedUpdate;

    // Variables for measured speed calculation
    unsigned long _lastMeasureTime;
    long _lastMeasureCount;
    double _measuredSpeedRPM;
};

#endif // DCMotorTacho_h

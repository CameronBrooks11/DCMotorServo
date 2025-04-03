#include "DCMotorTacho2.h"
#include <Arduino.h>

DCMotorTacho2::DCMotorTacho2(DCMotorServo *servo, double cpr, unsigned long speedInterval)
    : _servo(servo), _CPR(cpr), _speedInterval(speedInterval)
{
    _desiredSpeedRPM = 0;
    _speedPIDSetpoint = 0;
    _speedPIDInput = 0;
    _speedPIDOutput = 0;
    _measuredSpeedRPM = 0.0;
    _lastSpeedUpdate = millis();
    _lastEncoderCount = _servo->getActualPosition();

    // Create inner PID for speed control.
    // NOTE: We set output limits symmetrically to allow for negative outputs.
    _speedPID = new PID(&_speedPIDInput, &_speedPIDOutput, &_speedPIDSetpoint, 0.1, 0.2, 0.1, DIRECT);
    _speedPID->SetSampleTime(_speedInterval);
    _speedPID->SetOutputLimits(-_CPR * 10, _CPR * 10); // Default limits: ±CPR*10 counts.
    _speedPID->SetMode(AUTOMATIC);
}

void DCMotorTacho2::setSpeedRPM(double rpm)
{
    _desiredSpeedRPM = rpm;
    _speedPIDSetpoint = rpm;
    if (rpm == 0)
    {
        // If desired speed is zero, disable inner PID and engage brake.
        _speedPID->SetMode(MANUAL);
        _servo->stop();
    }
    else
    {
        // Otherwise, ensure the inner PID is in automatic mode.
        _speedPID->SetMode(AUTOMATIC);
    }
}

double DCMotorTacho2::getDesiredSpeedRPM() const
{
    return _desiredSpeedRPM;
}

void DCMotorTacho2::setSpeedPIDTunings(double Kp, double Ki, double Kd)
{
    _speedPID->SetTunings(Kp, Ki, Kd);
}

double DCMotorTacho2::getMeasuredSpeedRPM() const
{
    return _measuredSpeedRPM;
}

void DCMotorTacho2::run()
{
    // If desired speed is zero, ensure inner PID is off and stop motor.
    if (_desiredSpeedRPM == 0)
    {
        _speedPID->SetMode(MANUAL);
        _servo->stop();
        return;
    }

    unsigned long currentTime = millis();
    unsigned long dt = currentTime - _lastSpeedUpdate;
    if (dt >= _speedInterval)
    {
        long currentCount = _servo->getActualPosition();
        // Calculate measured speed in RPM.
        _measuredSpeedRPM = ((double)(currentCount - _lastEncoderCount) / _CPR) * (60000.0 / dt);
        _speedPIDInput = _measuredSpeedRPM;
        _speedPID->Compute();
        // Inner PID output (in counts) is added to the outer loop setpoint.
        _servo->move((long)_speedPIDOutput);
        _lastEncoderCount = currentCount;
        _lastSpeedUpdate = currentTime;
    }
    // Execute the outer (position) loop.
    _servo->run();
}

DCMotorServo *DCMotorTacho2::getServo() const
{
    return _servo;
}

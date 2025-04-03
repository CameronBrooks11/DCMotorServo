#include "DCMotorTacho_Legacy.h"
#include <Arduino.h>

DCMotorTacho_Legacy::DCMotorTacho_Legacy(MotorWriteFunc mWrite, MotorBrakeFunc mBrake, EncoderReadFunc eRead, EncoderWriteFunc eWrite,
                           double cpr, unsigned long speedInterval)
    : DCMotorServo(mWrite, mBrake, eRead, eWrite),
      _desiredSpeedRPM(0),
      _CPR(cpr),
      _speedInterval(speedInterval)
{
    unsigned long now = millis();
    _lastSpeedUpdate = now;
    _lastMeasureTime = now;
    _lastMeasureCount = getActualPosition();
    _measuredSpeedRPM = 0.0;
}

void DCMotorTacho_Legacy::setSpeedRPM(double rpm)
{
    _desiredSpeedRPM = rpm;
}

double DCMotorTacho_Legacy::getDesiredSpeedRPM() const
{
    return _desiredSpeedRPM;
}

void DCMotorTacho_Legacy::updateSpeedControl()
{
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - _lastSpeedUpdate;
    if (dt >= _speedInterval)
    {
        // Calculate the incremental counts to add to the setpoint.
        // desired RPM means (desired RPM/60000) rotations per ms.
        // Multiply by dt to get rotations over dt, then by CPR to get counts.
        double deltaCounts = (_desiredSpeedRPM * dt / 60000.0) * _CPR;
        // Update the setpoint using the inherited move() method (adds deltaCounts to _PID_setpoint)
        move((long)deltaCounts);
        _lastSpeedUpdate = currentTime;
    }
}

void DCMotorTacho_Legacy::updateMeasuredSpeed()
{
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - _lastMeasureTime;
    if (dt >= _speedInterval)
    {
        long currentCount = getActualPosition();
        // Compute measured speed in RPM:
        // (delta counts / CPR) gives rotations during dt ms. Multiply by 60000/dt to get RPM.
        _measuredSpeedRPM = ((double)(currentCount - _lastMeasureCount) / _CPR) * (60000.0 / dt);
        _lastMeasureCount = currentCount;
        _lastMeasureTime = currentTime;
    }
}

void DCMotorTacho_Legacy::runSpeed()
{
    updateSpeedControl();
    updateMeasuredSpeed();
    run(); // call base class PID loop run()
}

double DCMotorTacho_Legacy::getMeasuredSpeedRPM() const
{
    return _measuredSpeedRPM;
}

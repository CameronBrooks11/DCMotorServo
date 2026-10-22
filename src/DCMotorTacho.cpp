#include "DCMotorTacho.h"
#include <Arduino.h>

DCMotorTacho::DCMotorTacho(DCMotorServo *servo, double cpr, unsigned long speedInterval)
    : _servo(servo), _CPR(cpr), _speedInterval(speedInterval)
{
    _desiredSpeedRPM = 0;
    _speedPIDSetpoint = 0;
    _speedPIDInput = 0;
    _speedPIDOutput = 0;
    _measuredSpeedRPM = 0.0;
    _maxLead = 0;
    _lastSpeedUpdate = millis();
    _lastEncoderCount = _servo->getActualPosition();

    // Create inner PID for speed control.
    // NOTE: We set output limits symmetrically to allow for negative outputs.
    _speedPID = new PID(&_speedPIDInput, &_speedPIDOutput, &_speedPIDSetpoint, 0.1, 0.2, 0.1, DIRECT);
    _speedPID->SetSampleTime(_speedInterval);
    _speedPID->SetOutputLimits(-_CPR * 10, _CPR * 10); // Default limits: ±CPR*10 counts.
    _speedPID->SetMode(AUTOMATIC);
}

void DCMotorTacho::setSpeedRPM(double rpm)
{
    if (_desiredSpeedRPM == 0 && rpm != 0)
    {
        // Starting: measure from here, not from the last speed sample, which
        // may predate position or open-loop travel.
        _lastEncoderCount = _servo->getActualPosition();
        _lastSpeedUpdate = millis();
    }
    _desiredSpeedRPM = rpm;
    _speedPIDSetpoint = rpm;
    if (rpm == 0)
    {
        // If desired speed is zero, disable inner PID and engage brake.
        holdSpeedLoop();
        _servo->stop();
    }
    else
    {
        // Otherwise, ensure the inner PID is in automatic mode.
        _speedPID->SetMode(AUTOMATIC);
    }
}

double DCMotorTacho::getDesiredSpeedRPM() const
{
    return _desiredSpeedRPM;
}

void DCMotorTacho::setPIDTunings(double Kp, double Ki, double Kd)
{
    _speedPID->SetTunings(Kp, Ki, Kd);
}

void DCMotorTacho::setMaxLead(long counts)
{
    _maxLead = (counts < 0) ? -counts : counts;
}

double DCMotorTacho::getMeasuredSpeedRPM() const
{
    return _measuredSpeedRPM;
}

void DCMotorTacho::run()
{
    // Hold the speed loop while the servo is homing or stall-latched: winding
    // the position setpoint against a frozen encoder would make clearStall()
    // (or homing completion) a runaway move, and the zero-speed stop() below
    // would silently cancel an in-progress homing move.
    if (_servo->isHoming() || _servo->isStalled())
    {
        holdSpeedLoop();
        _servo->run(); // progress homing / hold the latched brake
        // Sync AFTER run(): homing completion zeroes the encoder inside it,
        // and a pre-run() count would fake a huge speed sample next cycle.
        _lastEncoderCount = _servo->getActualPosition();
        _lastSpeedUpdate = millis();
        return;
    }

    // If desired speed is zero, ensure inner PID is off and stop motor.
    if (_desiredSpeedRPM == 0)
    {
        holdSpeedLoop();
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
        // Re-arm in case the hold above put the PID in MANUAL (no-op when
        // already AUTOMATIC; the MANUAL->AUTOMATIC transition reinitializes).
        _speedPID->SetMode(AUTOMATIC);
        _speedPID->Compute();
        // Inner PID output (in counts) is added to the outer loop setpoint.
        _servo->move((long)_speedPIDOutput);
        if (_maxLead > 0)
        {
            // Don't let a stalled shaft bank travel for a lunge on release.
            long lead = _servo->getRequestedPosition() - currentCount;
            if (lead > _maxLead)
                _servo->moveTo(currentCount + _maxLead);
            else if (lead < -_maxLead)
                _servo->moveTo(currentCount - _maxLead);
        }
        _lastEncoderCount = currentCount;
        _lastSpeedUpdate = currentTime;
    }
    // Execute the outer (position) loop.
    _servo->run();
}

/**
 *  Stops the motor and disables the inner PID controller.
 */
void DCMotorTacho::stop()
{
    _desiredSpeedRPM = _speedPIDSetpoint = 0;
    _servo->stop();
    holdSpeedLoop();
}

void DCMotorTacho::holdSpeedLoop()
{
    // The next MANUAL->AUTOMATIC transition seeds the integral from the
    // output and the derivative from the input. The motor is braked or
    // homing whenever the loop is held, so restart both from rest.
    _speedPID->SetMode(MANUAL);
    _speedPIDOutput = 0;
    _speedPIDInput = 0;
}

DCMotorServo *DCMotorTacho::getServo() const
{
    return _servo;
}

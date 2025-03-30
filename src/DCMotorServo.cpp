// DCMotorServo.cpp
#include "DCMotorServo.h"
#include <Arduino.h>

DCMotorServo::DCMotorServo(MotorWriteFunc mWrite, MotorBrakeFunc mBrake, EncoderReadFunc eRead, EncoderWriteFunc eWrite)
    : motorWrite(mWrite), motorBrake(mBrake), encoderRead(eRead), encoderWrite(eWrite), _maxPWM(255)
{
  _PWM_output = 0;
  _pwm_skip = 50;
  _position_accuracy = 30;

  _PID_input = encoderRead();
  _PID_output = 0;
  _PID_setpoint = _PID_input;

  myPID = new PID(&_PID_input, &_PID_output, &_PID_setpoint, 0.1, 0.2, 0.1, DIRECT);

  // Configure PID controller
  myPID->SetSampleTime(50);
  myPID->SetOutputLimits(_pwm_skip - 255, 255 - _pwm_skip);
  myPID->SetMode(AUTOMATIC);
}

void DCMotorServo::setCurrentPosition(int new_position)
{
  encoderWrite(new_position);
  _PID_input = encoderRead();
}

void DCMotorServo::setAccuracy(unsigned int range)
{
  _position_accuracy = range;
}

bool DCMotorServo::setPWMSkip(uint8_t range)
{
  if (range < 255)
  {
    _pwm_skip = range;
    return true;
  }
  return false;
}

void DCMotorServo::setMaxPWM(uint8_t maxPWM)
{
  _maxPWM = maxPWM;
  myPID->SetOutputLimits(_pwm_skip - _maxPWM, _maxPWM - _pwm_skip);
}

void DCMotorServo::SetPIDTunings(double Kp, double Ki, double Kd)
{
  myPID->SetTunings(Kp, Ki, Kd);
}

bool DCMotorServo::finished()
{
  return abs(_PID_setpoint - _PID_input) < _position_accuracy && _PWM_output == 0;
}

void DCMotorServo::move(int new_rela_position)
{
  _PID_setpoint += new_rela_position;
}

void DCMotorServo::moveTo(int new_position)
{
  _PID_setpoint = new_position;
}

int DCMotorServo::getRequestedPosition()
{
  return _PID_setpoint;
}

int DCMotorServo::getActualPosition()
{
  return encoderRead();
}

void DCMotorServo::run()
{
  _PID_input = encoderRead();
  myPID->Compute();

  // Compute a PWM value that adds a base skip value
  int outputPWM = constrain(abs(_PID_output) + _pwm_skip, _pwm_skip, _maxPWM);

  if (abs(_PID_setpoint - _PID_input) < _position_accuracy)
  {
    myPID->SetMode(MANUAL);
    _PID_output = 0;
    outputPWM = 0;
  }
  else
  {
    myPID->SetMode(AUTOMATIC);
  }

  // Determine final speed (direction based on PID output sign)
  int finalSpeed = (_PID_output < 0) ? -outputPWM : outputPWM;

  // Send the computed speed via the user-supplied function
  motorWrite(finalSpeed);
}

void DCMotorServo::stop()
{
  myPID->SetMode(MANUAL);
  _PID_output = 0;
  _PWM_output = 0;
  motorBrake();
}

String DCMotorServo::getDebugInfo()
{
  String info = "DCMotorServo Debug Info:\n";
  info += "Encoder Position: " + String(encoderRead()) + "\n";
  info += "PID Setpoint: " + String(_PID_setpoint) + "\n";
  info += "PID Input: " + String(_PID_input) + "\n";
  info += "PID Output: " + String(_PID_output) + "\n";
  info += "PWM Skip: " + String(_pwm_skip) + "\n";
  info += "Position Accuracy: " + String(_position_accuracy) + "\n";

  double Kp = myPID->GetKp();
  double Ki = myPID->GetKi();
  double Kd = myPID->GetKd();
  info += "PID Params - Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd: " + String(Kd) + "\n";

  return info;
}

String DCMotorServo::getSerialPlotter()
{
  String data = "Setpoint:" + String(_PID_setpoint) + ", Input:" + String(_PID_input) + ", Output:" + String(_PID_output);
  return data;
}

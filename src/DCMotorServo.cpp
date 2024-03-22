#include <DCMotorServo.h>
#include <Arduino.h> // Required for string support in debug function

/**
 * Constructor for DCMotorServo. Initializes pins, encoder, and PID controller with default values.
 * @param pin_dir_1 Digital pin for direction control 1.
 * @param pin_dir_2 Digital pin for direction control 2.
 * @param pin_PWM_output Digital pin for PWM output to motor.
 * @param pin_encode1 Digital pin for encoder input 1.
 * @param pin_encode2 Digital pin for encoder input 2.
 */
DCMotorServo::DCMotorServo(uint8_t pin_dir_1, uint8_t pin_dir_2, uint8_t pin_PWM_output, uint8_t pin_encode1, uint8_t pin_encode2)
: _maxPWM(255){
  _pin_PWM_output = pin_PWM_output;
  _pin_dir_1 = pin_dir_1;
  _pin_dir_2 = pin_dir_2;

  // Initialize pins for direction and PWM output
  pinMode(_pin_dir_1, OUTPUT);
  pinMode(_pin_dir_2, OUTPUT);
  pinMode(_pin_PWM_output, OUTPUT);

  // Initialize encoder and PID variables
  _position = new Encoder(pin_encode1, pin_encode2);
  _PWM_output = 0;
  _pwm_skip = 50;
  _position_accuracy = 30;

  _PID_input = _position->read();
  _PID_output = 0;
  _PID_setpoint = _PID_input;
  myPID = new PID(&_PID_input, &_PID_output, &_PID_setpoint, .1, .2, .1, DIRECT);

  // Configure PID
  myPID->SetSampleTime(50);
  myPID->SetOutputLimits(_pwm_skip - 255, 255 - _pwm_skip);
  myPID->SetMode(AUTOMATIC);
}

/**
 * Sets the current motor position.
 * @param new_position The new position to set the motor to.
 */
void DCMotorServo::setCurrentPosition(int new_position)
{
  _position->write(new_position);
  _PID_input = _position->read();
}

/**
 * Sets the accuracy for determining if the motor has reached its target position.
 * @param range The acceptable range of error from the target position.
 */
void DCMotorServo::setAccuracy(unsigned int range)
{
  _position_accuracy = range;
}

/**
 * Sets the range of PWM values to skip to avoid stalling the motor at low speeds.
 * @param range The minimum PWM value to apply to the motor.
 * @return True if range is within valid bounds (0-254), false otherwise.
 */
bool DCMotorServo::setPWMSkip(uint8_t range)
{
  if (0 <= range && range < 255)
  {
    _pwm_skip = range;
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * Sets the maximum allowable PWM value to control the motor speed.
 * @param maxPWM The maximum PWM value, which caps the speed.
 */
void DCMotorServo::setMaxPWM(uint8_t maxPWM)
{
  _maxPWM = maxPWM;
  myPID->SetOutputLimits(_pwm_skip - _maxPWM, _maxPWM - _pwm_skip); // Update the PID output limits
}

/**
 * Sets the PID tuning parameters.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 */
void DCMotorServo::SetPIDTunings(double Kp, double Ki, double Kd)
{
  myPID->SetTunings(Kp, Ki, Kd);
}

/**
 * Checks if the motor has reached its target position within the specified accuracy.
 * @return True if the motor has reached its target, false otherwise.
 */
bool DCMotorServo::finished()
{
  return abs(_PID_setpoint - _PID_input) < _position_accuracy && _PWM_output == 0;
}

/**
 * Moves the motor to a new relative position from the current position.
 * @param new_rela_position The relative position to move to.
 */
void DCMotorServo::move(int new_rela_position)
{
  _PID_setpoint += new_rela_position;
}

/**
 * Moves the motor to an absolute position.
 * @param new_position The target position for the motor.
 */
void DCMotorServo::moveTo(int new_position)
{
  _PID_setpoint = new_position;
}

/**
 * Retrieves the current target position of the motor.
 * @return The motor's target position.
 */
int DCMotorServo::getRequestedPosition()
{
  return _PID_setpoint;
}

/**
 * Retrieves the actual current position of the motor from the encoder.
 * @return The current position as read from the encoder.
 */
int DCMotorServo::getActualPosition()
{
  return _position->read();
}


/**
 * Runs the PID controller to move the motor towards the target position.
 * Adjusts the motor's PWM based on PID output and ensures it does not exceed _maxPWM.
 */
void DCMotorServo::run()
{
  _PID_input = _position->read();
  myPID->Compute();
  
  // Ensure the maximum PWM cap, shouldnt be necessary but just in case
  _PWM_output = constrain(abs(_PID_output) + _pwm_skip, _pwm_skip, _maxPWM);

  if (abs(_PID_setpoint - _PID_input) < _position_accuracy)
  {
    myPID->SetMode(MANUAL);
    _PID_output = 0;
    _PWM_output = 0;
  }
  else
  {
    myPID->SetMode(AUTOMATIC);
  }

  _pick_direction();
  analogWrite(_pin_PWM_output, _PWM_output);
}

/**
 * Stops the motor by setting PID to manual mode and disabling PWM output.
 */
void DCMotorServo::stop()
{
  myPID->SetMode(MANUAL);
  _PID_output = 0;
  _PWM_output = 0;
  analogWrite(_pin_PWM_output, _PWM_output);
}

/**
 * Retrieves debugging information including encoder position and PID parameters.
 * @return String containing debug information.
 */
String DCMotorServo::getDebugInfo()
{
  String info = "DCMotorServo Debug Info:\n";
  info += "Encoder Position: " + String(_position->read()) + "\n";
  info += "PID Setpoint: " + String(_PID_setpoint) + "\n";
  info += "PID Input: " + String(_PID_input) + "\n";
  info += "PID Output: " + String(_PID_output) + "\n";
  info += "PWM Output: " + String(_PWM_output) + "\n";
  info += "PWM Skip: " + String(_pwm_skip) + "\n";
  info += "Position Accuracy: " + String(_position_accuracy) + "\n";

  // Extract PID parameters
  double Kp, Ki, Kd;
  Kp=myPID->GetKp();
  Ki=myPID->GetKi();
  Kd=myPID->GetKd();
  info += "PID Params - Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd: " + String(Kd) + "\n";

  return info;
}

/**
 * Generates a string formatted for the Serial Plotter in Arduino IDE, showing
 * current PID values with labels for real-time tuning.
 * @return A string of labeled PID values: Setpoint, Input, and Output.
 */
String DCMotorServo::getSerialPlotter() {
  String data = "Setpoint:" + String(_PID_setpoint) + ", Input:" + String(_PID_input) + ", Output:" + String(_PID_output);
  return data;
}


/**
 * Determines the direction of motor rotation based on the sign of PID output.
 */
void DCMotorServo::_pick_direction()
{
  if (_PID_output < 0)
  {
    analogWrite(_pin_dir_1, LOW);
    digitalWrite(_pin_dir_2, HIGH);
  }
  else
  {
    digitalWrite(_pin_dir_1, HIGH);
    digitalWrite(_pin_dir_2, LOW);
  }
}

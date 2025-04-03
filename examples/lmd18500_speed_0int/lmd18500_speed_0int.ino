/*
 * Example: DCMotorServo with LMD18200 driver
 *
 * LMD18200 Motor Driver Pin Assignment:
 *   PWM: 6, DIR: 7, BRAKE: 8
 * Encoder Pin Assignment:
 *   Channel B1: A2, Channel A1: A3
 */

#include <Arduino.h>
#include <Encoder.h>
#include "LMD18200.h"
#include "DCMotorServo.h"

// Define the motor and encoder pins
#define MOTOR_PWM_PIN 6
#define MOTOR_DIR_PIN 7
#define MOTOR_BRAKE_PIN 8
#define ENCODER_PIN1 A2
#define ENCODER_PIN2 A3

// Create an instance of the LMD18200 motor driver
LMD18200 motorDriver(MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_BRAKE_PIN);

// Create an instance of the Encoder
Encoder myEncoder(ENCODER_PIN1, ENCODER_PIN2);

// Wrapper functions for the motor driver
void lmdMotorWrite(int16_t speed)
{
    motorDriver.write(speed);
}

void lmdMotorBrake()
{
    motorDriver.brake();
}

// Wrapper functions for the encoder
int encoderReadFunc()
{
    return myEncoder.read();
}

void encoderWriteFunc(int newPosition)
{
    myEncoder.write(newPosition);
}

// Create an instance of DCMotorServo using the function pointers
DCMotorServo servo(lmdMotorWrite, lmdMotorBrake, encoderReadFunc, encoderWriteFunc);

// PID parameters for tuning
float KP = 0.15;
float KI = 0.00;
float KD = 0.001;

// ------------------------
// Physical specifications
// ------------------------

// ----- Positional specifications -----

// MAX PWM value for the motor driver, from driver datasheet
#define MAX_PWM 255 // Maximum PWM value for the motor driver

// MIN PWM value to skip over based on BDC motor characteristics
// This represents the minimum PWM value required to overcome the static friction of the motor.
// This value is motor-specific and should be determined experimentally. This will in general be
// a low value, around 5-10% of the max PWM value, but will be slightly higher for large and/or
// gear-reduced motors.
// This is obtained from testing the motor driver with the motor. Run the script.
#define MIN_PWM 25 // Minimum PWM value to skip over (motor-specific)

// Accuracy of the PID controller
// This is the accuracy of the PID controller. It is the minimum difference between the setpoint and input
// that will cause the PID controller to update the output. Essentially, this is the minimum error that we will tolerate
// before we update the output. This is a trade-off between system accuracy and stability. A smaller value will give a more
// accurate system but decrease stability. A larger value will give a more stable system but less accuracy.

// Determine starting point somehow then tune PID then see how low you can go.
#define ACCURACY 10 // Accuracy of the PID controller

// ------- Speed specifications -------

// Obtained from manufacturer's datasheet
#define PPR 12        // Pulses per revolution (PPR) of the encoder
#define GEAR_RATIO 19 // Gear ratio of the gearbox. Change this according to your motor's gear ratio.

// Obtained from testing
#define EMPIRICAL_FUDGE_FACTOR 0.875 // Empirical fudge factor for tuning, obtained from testing

// Calculation of physical parameters
#define ENCODER_RESOLUTION (PPR * GEAR_RATIO)                 // Encoder resolution (counts per revolution)
#define CPR (ENCODER_RESOLUTION * 4 * EMPIRICAL_FUDGE_FACTOR) // Counts per revolution (CPR) for the motor

// Define a fixed interval (in milliseconds) for speed calculation
// This is required to avoid taking readings too frequently and
// to allow for smoother speed calculations. When just read directly
// from the loop, the speed will be VERY (entirely) noisy / erratic.
// Essentially we are averaging the speed over this interval.
// This is a trade-off between speed calculation accuracy and responsiveness.
// A smaller value will give a more responsive system but less accurate speed readings.
// A larger value will give a more accurate speed reading but less responsiveness.
// In general, systems using a BDC motor should not be expecting high precision or responsiveness
// in the first place, so erring on the side of a larger value is preferred. 5ms is a good starting point.
#define speedCalcInterval 5

// Variables for speed calculation
static long lastEncoderCount = 0;
static unsigned long lastSpeedTime = 0;
double speedRPM = 0.0;

// -----------------------
// Serial specifications
// -----------------------

#define SERIALO_INTERVAL 50 // Interval for serial output in milliseconds

void setup()
{
    Serial.begin(115200);

    // Initialize the motor driver
    motorDriver.begin();

    // Configure the PID controller
    servo.myPID->SetTunings(KP, KI, KD);
    servo.setPWMSkip(MIN_PWM);
    servo.setMaxPWM(MAX_PWM);
    servo.setAccuracy(ACCURACY);

    Serial.println("DCMotorServo with LMD18200 Example");
    Serial.println("Setpoint,Input,Output,Speed(RPM)");
    // Initialize speed calculation variables
    lastEncoderCount = myEncoder.read();
    lastSpeedTime = millis();
}

void loop()
{

    // Run the PID loop
    servo.run();

    // Calculate speed
    calculateSpeed();

    // Serial input/output
    serialIO();
}

void calculateSpeed()
{
    // Get the current time
    unsigned long currentTime = millis();

    // Calculate speed only every speedCalcInterval ms
    long currentEncoderCount = servo.getActualPosition();
    unsigned long dt = currentTime - lastSpeedTime;
    if (dt >= speedCalcInterval)
    {
        // Calculate RPM:
        // (delta counts / CPR) gives rotations over dt ms.
        // Multiply by 60000/dt to get RPM.
        speedRPM = ((double)(currentEncoderCount - lastEncoderCount) / (double)CPR) * (60000.0 / dt);
        lastEncoderCount = currentEncoderCount;
        lastSpeedTime = currentTime;
    }
}

void serialIO()
{

    // Declare a static variable to store the last print time
    static unsigned long lastPrintTime = millis();

    // OUTPUT
    if (millis() - lastPrintTime > SERIALO_INTERVAL)
    {
        lastPrintTime = millis();
        Serial.print(servo.getSerialPlotter());
        Serial.print(", Speed(RPM):");
        Serial.println(speedRPM, 2); // Print speed with 2 decimal places
    }

    // INPUT
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("KP="))
        {
            KP = command.substring(3).toFloat();
            servo.myPID->SetTunings(KP, KI, KD);
            Serial.print("KP set to: ");
            Serial.println(KP);
        }
        else if (command.startsWith("KI="))
        {
            KI = command.substring(3).toFloat();
            servo.myPID->SetTunings(KP, KI, KD);
            Serial.print("KI set to: ");
            Serial.println(KI);
        }
        else if (command.startsWith("KD="))
        {
            KD = command.substring(3).toFloat();
            servo.myPID->SetTunings(KP, KI, KD);
            Serial.print("KD set to: ");
            Serial.println(KD);
        }
        else if (command.startsWith("MOVE="))
        {
            int newPosition = command.substring(5).toInt();
            if (newPosition != 0)
            {
                servo.move(newPosition);
                Serial.print("Moving to: ");
                Serial.println(newPosition);
            }
            else
            {
                Serial.println("Invalid MOVE command");
            }
        }
        else if (command.startsWith("MAXPWM="))
        {
            int maxPWM = command.substring(7).toInt();
            if (maxPWM >= 0 && maxPWM <= 255)
            {
                servo.setMaxPWM(maxPWM);
                Serial.print("Setting MAXPWM to: ");
                Serial.println(maxPWM);
            }
            else
            {
                Serial.println("Invalid MAXPWM command");
            }
        }
        else if (command.startsWith("MOVEROTATE="))
        {
            int numRotations = command.substring(11).toInt();
            if (numRotations != 0)
            {
                servo.move(numRotations * CPR);
                Serial.print("Rotating ");
                Serial.print(numRotations);
                Serial.println(" full revolutions");
            }
            else
            {
                Serial.println("Invalid MOVEROTATE command");
            }
        }
        else
        {
            Serial.println("Invalid command");
        }
    }
}
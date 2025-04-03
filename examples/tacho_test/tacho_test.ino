/*
 * Example: DCMotorTacho with LMD18200 driver for Speed Control
 *
 * LMD18200 Motor Driver Pin Assignment:
 *   PWM: 6, DIR: 7, BRAKE: 8
 * Encoder Pin Assignment:
 *   Channel B1: A2, Channel A1: A3
 */

#include <Arduino.h>
#include <Encoder.h>
#include "LMD18200.h"
#include "DCMotorTacho.h"

// Define motor and encoder pins
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
long encoderReadFunc()
{
    return myEncoder.read();
}
void encoderWriteFunc(long newPosition)
{
    myEncoder.write(newPosition);
}

// Define physical parameters for the encoder & gearbox:
#define PPR 12        // Pulses per revolution of the encoder
#define GEAR_RATIO 19 // Gear ratio of the gearbox
#define EMPIRICAL_FUDGE_FACTOR 0.875
#define ENCODER_RESOLUTION (PPR * GEAR_RATIO)
#define CPR (ENCODER_RESOLUTION * 4 * EMPIRICAL_FUDGE_FACTOR) // counts per revolution

// Create an instance of DCMotorTacho using the function pointers and physical parameters
DCMotorTacho tacho(lmdMotorWrite, lmdMotorBrake, encoderReadFunc, encoderWriteFunc, CPR, 50);

// PID parameters for tuning (for position control underlying speed control)
float KP = 0.15, KI = 0.00, KD = 0.001;

// Speed command: desired speed in RPM
double desiredRPM = 100; // for example, 100 RPM

// Serial update interval
#define SERIAL_INTERVAL 50

void setup()
{
    Serial.begin(115200);
    motorDriver.begin();

    // Configure the PID controller (inherited from DCMotorServo)
    tacho.myPID->SetTunings(KP, KI, KD);
    tacho.setPWMSkip(25);
    tacho.setAccuracy(10);
    tacho.setMaxPWM(255);

    // Set initial desired speed
    tacho.setSpeedRPM(desiredRPM);

    Serial.println("DCMotorTacho Example (Speed Control)");
    Serial.println("Setpoint, Input, Output, Measured Speed (RPM)");
}

void loop()
{
    static unsigned long lastPrintTime = millis();

    // Update speed control and run the PID loop
    tacho.runSpeed();

    // Print speed and PID data at a slower rate for readability
    if (millis() - lastPrintTime > SERIAL_INTERVAL)
    {
        lastPrintTime = millis();
        Serial.print(tacho.getSerialPlotter());
        Serial.print(", Measured Speed (RPM): ");
        Serial.println(tacho.getMeasuredSpeedRPM(), 2);
    }

    // Serial commands for tuning or speed adjustment can be added here.
    // For example, you might allow:
    // "SPEED=120" to change the desired speed to 120 RPM.
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("SPEED="))
        {
            double newRPM = command.substring(6).toFloat();
            tacho.setSpeedRPM(newRPM);
            Serial.print("Desired speed set to: ");
            Serial.println(newRPM);
        }
        // Additional PID tuning commands (KP, KI, KD) can be parsed similarly.
        else if (command.startsWith("KP="))
        {
            KP = command.substring(3).toFloat();
            tacho.myPID->SetTunings(KP, KI, KD);
            Serial.print("KP set to: ");
            Serial.println(KP);
        }
        else if (command.startsWith("KI="))
        {
            KI = command.substring(3).toFloat();
            tacho.myPID->SetTunings(KP, KI, KD);
            Serial.print("KI set to: ");
            Serial.println(KI);
        }
        else if (command.startsWith("KD="))
        {
            KD = command.substring(3).toFloat();
            tacho.myPID->SetTunings(KP, KI, KD);
            Serial.print("KD set to: ");
            Serial.println(KD);
        }
        else
        {
            Serial.println("Invalid command");
        }
    }
}

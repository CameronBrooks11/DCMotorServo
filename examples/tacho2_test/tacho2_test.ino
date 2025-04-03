/*
 * Example: DCMotorTacho2 (Cascaded Speed Control) with LMD18200 driver
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
#include "DCMotorTacho2.h"

// Define motor and encoder pins.
#define MOTOR_PWM_PIN 6
#define MOTOR_DIR_PIN 7
#define MOTOR_BRAKE_PIN 8
#define ENCODER_PIN1 A2
#define ENCODER_PIN2 A3

// Create an instance of the LMD18200 motor driver.
LMD18200 motorDriver(MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_BRAKE_PIN);

// Create an instance of the Encoder.
Encoder myEncoder(ENCODER_PIN1, ENCODER_PIN2);

// Wrapper functions for the motor driver.
void lmdMotorWrite(int16_t speed)
{
    motorDriver.write(speed);
}
void lmdMotorBrake()
{
    motorDriver.brake();
}

// Wrapper functions for the encoder.
long encoderReadFunc()
{
    return myEncoder.read();
}
void encoderWriteFunc(long newPosition)
{
    myEncoder.write(newPosition);
}

// Create an instance of DCMotorServo using the wrappers.
DCMotorServo servo(lmdMotorWrite, lmdMotorBrake, encoderReadFunc, encoderWriteFunc);

// Physical parameters for speed control.
#define PPR 12
#define GEAR_RATIO 19
#define EMPIRICAL_FUDGE_FACTOR 0.875
#define ENCODER_RESOLUTION (PPR * GEAR_RATIO)
#define CPR (ENCODER_RESOLUTION * 4 * EMPIRICAL_FUDGE_FACTOR)

// Create an instance of DCMotorTacho2.
// For inner PID (speed control), we use inner PID tunings (example values).
DCMotorTacho2 tacho(&servo, CPR, 20); // speedInterval = 20 ms

// Outer loop PID tunings (position control) for the underlying servo.
float outerKp = 0.15, outerKi = 0.00, outerKd = 0.001;
// Inner loop PID tunings (speed control) will be adjusted via serial commands.
float innerKp = 0.6, innerKi = 0.2, innerKd = 0.01;

void setup()
{
    Serial.begin(115200);
    // Initialize the motor driver.
    motorDriver.begin();

    // Configure the outer (position) PID controller.
    servo.SetPIDTunings(outerKp, outerKi, outerKd);
    servo.setPWMSkip(25);
    servo.setAccuracy(10);
    servo.setMaxPWM(255);

    tacho.setSpeedRPM(0);
    tacho.setSpeedPIDTunings(innerKp, innerKi, innerKd);

    Serial.println("DCMotorTacho2 (Cascaded Speed Control) Example");
    Serial.println("Format: {Outer: Setpoint,Input,Output} | MeasuredSpeedRPM | DesiredSpeedRPM");
}

void loop()
{
    static unsigned long lastPrintTime = millis();

    // Run the cascaded control loop.
    tacho.run();

    // Print status every 50ms.
    if (millis() - lastPrintTime > 50)
    {
        lastPrintTime = millis();
        double measuredSpeed = tacho.getMeasuredSpeedRPM();
        double desiredSpeed = tacho.getDesiredSpeedRPM();
        Serial.print(tacho.getServo()->getSerialPlotter());
        Serial.print(", MeasuredSpeedRPM:");
        Serial.print(measuredSpeed, 2);
        Serial.print(", DesiredSpeedRPM:");
        Serial.println(desiredSpeed, 2);
    }

    // Process serial commands for tuning.
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("SPEED="))
        {
            double rpm = command.substring(6).toFloat();
            tacho.setSpeedRPM(rpm);
            Serial.print("Desired Speed set to: ");
            Serial.println(rpm);
        }
        else if (command.startsWith("SPEEDKP="))
        {
            double kp = command.substring(8).toFloat();
            tacho.setSpeedPIDTunings(kp, innerKi, innerKd);
            innerKp = kp;
            Serial.print("Inner PID Kp set to: ");
            Serial.println(kp);
        }
        else if (command.startsWith("SPEEDKI="))
        {
            double ki = command.substring(8).toFloat();
            tacho.setSpeedPIDTunings(innerKp, ki, innerKd);
            innerKi = ki;
            Serial.print("Inner PID Ki set to: ");
            Serial.println(ki);
        }
        else if (command.startsWith("SPEEDKD="))
        {
            double kd = command.substring(8).toFloat();
            tacho.setSpeedPIDTunings(innerKp, innerKi, kd);
            innerKd = kd;
            Serial.print("Inner PID Kd set to: ");
            Serial.println(kd);
        }
        // Outer loop tuning commands.
        else if (command.startsWith("KP="))
        {
            outerKp = command.substring(3).toFloat();
            servo.myPID->SetTunings(outerKp, outerKi, outerKd);
            Serial.print("Outer PID Kp set to: ");
            Serial.println(outerKp);
        }
        else if (command.startsWith("KI="))
        {
            outerKi = command.substring(3).toFloat();
            servo.myPID->SetTunings(outerKp, outerKi, outerKd);
            Serial.print("Outer PID Ki set to: ");
            Serial.println(outerKi);
        }
        else if (command.startsWith("KD="))
        {
            outerKd = command.substring(3).toFloat();
            servo.myPID->SetTunings(outerKp, outerKi, outerKd);
            Serial.print("Outer PID Kd set to: ");
            Serial.println(outerKd);
        }
        else
        {
            Serial.println("Invalid command. Commands: SPEED=, SPEEDKP=, SPEEDKI=, SPEEDKD=, KP=, KI=, KD=");
        }
    }
}

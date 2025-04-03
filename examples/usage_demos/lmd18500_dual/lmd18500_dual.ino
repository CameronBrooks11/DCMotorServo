/*
 * Example: Dual DCMotorServo with LMD18200 driver
 *
 * LMD18200 Motor Driver Pin Assignment:
 * - Motor1: PWM: 6, DIR: 7, BRAKE: 8
 * - Motor2: PWM: 9, DIR: 10, BRAKE: 11
 *
 * Encoder Pin Assignment:
 * - Motor1: Channel B1: A2, Channel A1: A3
 * - Motor2: Channel B2: A6, Channel A1: A7
 */

#include <Arduino.h>
#include <Encoder.h>
#include "LMD18200.h"
#include "DCMotorServo.h"

// ----- Motor1 Definitions -----
#define MOTOR1_PWM_PIN 6
#define MOTOR1_DIR_PIN 7
#define MOTOR1_BRAKE_PIN 8
#define ENCODER1_PIN1 A2
#define ENCODER1_PIN2 A3

// ----- Motor2 Definitions -----
#define MOTOR2_PWM_PIN 11
#define MOTOR2_DIR_PIN 10
#define MOTOR2_BRAKE_PIN 12
#define ENCODER2_PIN1 A0
#define ENCODER2_PIN2 A1

// Create LMD18200 instances for both motors
LMD18200 motor1(MOTOR1_PWM_PIN, MOTOR1_DIR_PIN, MOTOR1_BRAKE_PIN);
LMD18200 motor2(MOTOR2_PWM_PIN, MOTOR2_DIR_PIN, MOTOR2_BRAKE_PIN);

// Create Encoder instances for both motors
Encoder encoder1(ENCODER1_PIN1, ENCODER1_PIN2);
Encoder encoder2(ENCODER2_PIN1, ENCODER2_PIN2);

// ----- Wrapper Functions for Motor1 -----
void lmdMotor1Write(int16_t speed)
{
    motor1.write(speed);
}
void lmdMotor1Brake()
{
    motor1.brake();
}
int encoder1ReadFunc()
{
    return encoder1.read();
}
void encoder1WriteFunc(int newPosition)
{
    encoder1.write(newPosition);
}

// ----- Wrapper Functions for Motor2 -----
void lmdMotor2Write(int16_t speed)
{
    motor2.write(speed);
}
void lmdMotor2Brake()
{
    motor2.brake();
}
int encoder2ReadFunc()
{
    return encoder2.read();
}
void encoder2WriteFunc(int newPosition)
{
    encoder2.write(newPosition);
}

// Create two DCMotorServo instances using the respective function pointers
DCMotorServo servo1(lmdMotor1Write, lmdMotor1Brake, encoder1ReadFunc, encoder1WriteFunc);
DCMotorServo servo2(lmdMotor2Write, lmdMotor2Brake, encoder2ReadFunc, encoder2WriteFunc);

// PID parameters for tuning
float KP1 = 0.1, KI1 = 0.15, KD1 = 0.1;
float KP2 = 0.1, KI2 = 0.15, KD2 = 0.01;

void setup()
{
    Serial.begin(115200);

    // Initialize motor drivers
    motor1.begin();
    Serial.println("LMD18200 Motor1 initialized");
    motor2.begin();
    Serial.println("LMD18200 Motor2 initialized");

    // Configure PID for servo1 (Motor1)
    servo1.myPID->SetTunings(KP1, KI1, KD1);
    servo1.setPWMSkip(25);
    servo1.setAccuracy(14);
    servo1.setMaxPWM(200); // Set max PWM for servo1

    // Configure PID for servo2 (Motor2)
    servo2.myPID->SetTunings(KP2, KI2, KD2);
    servo2.setPWMSkip(25);
    servo2.setAccuracy(14);
    servo2.setMaxPWM(150); // Set max PWM for servo2

    Serial.println("Dual DCMotorServo with LMD18200 Example");
    Serial.println("Format: Setpoint, Input, Output");
}

void loop()
{
    static unsigned long lastPrintTime = millis();

    // Run PID loops for both servos
    servo1.run();
    servo2.run();

    // Print PID values every 20ms for tuning feedback
    if (millis() - lastPrintTime > 20)
    {
        lastPrintTime = millis();
        Serial.print(servo1.getSerialPlotter(1));
        Serial.print(", ");
        Serial.println(servo2.getSerialPlotter(2));
    }

    // Process Serial commands for PID tuning or movement
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("MOVE1="))
        {
            int newPosition = command.substring(6).toInt();
            if (newPosition != 0)
            {
                servo1.move(newPosition);
                Serial.print("Servo1 moving to: ");
                Serial.println(newPosition);
            }
            else
            {
                Serial.println("Invalid MOVE1 command");
            }
        }
        else if (command.startsWith("MOVE2="))
        {
            int newPosition = command.substring(6).toInt();
            if (newPosition != 0)
            {
                servo2.move(newPosition);
                Serial.print("Servo2 moving to: ");
                Serial.println(newPosition);
            }
            else
            {
                Serial.println("Invalid MOVE2 command");
            }
        }
        else
        {
            Serial.println("Invalid command. Use MOVE1=<value> or MOVE2=<value>");
        }
    }
}

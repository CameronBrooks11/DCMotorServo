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

/*
// First motor driver example
#define MOTOR_PWM_PIN 6
#define MOTOR_DIR_PIN 7
#define MOTOR_BRAKE_PIN 8
#define ENCODER_PIN1 A2
#define ENCODER_PIN2 A3
*/
/*
// Second motor driver example
#define MOTOR_PWM_PIN 11
#define MOTOR_DIR_PIN 10
#define MOTOR_BRAKE_PIN 12
#define ENCODER_PIN1 A0
#define ENCODER_PIN2 A1
*/

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
float KP = 0.1;
float KI = 0.15;
float KD = 0.05;

void setup()
{
    Serial.begin(115200);

    // Initialize the motor driver
    motorDriver.begin();

    // Configure the PID controller
    servo.myPID->SetTunings(KP, KI, KD);
    servo.setPWMSkip(25);
    servo.setAccuracy(14);
    servo.setMaxPWM(200);

    Serial.println("DCMotorServo with LMD18200 Example");
    Serial.println("Setpoint,Input,Output");
}

void loop()
{
    static unsigned long lastPrintTime = millis();

    // Run the PID loop
    servo.run();

    // Print PID values for tuning via Serial Plotter every 20ms
    if (millis() - lastPrintTime > 20)
    {
        lastPrintTime = millis();
        Serial.println(servo.getSerialPlotter());
    }

    // Allow PID tuning commands via Serial
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
        else
        {
            Serial.println("Invalid command");
        }
    }
}

/*
 * Example: DCMotorServo with L298N driver
 *
 * L298N Motor Driver Pin Assignment:
 *   ENA (PWM): 3, IN1: 4, IN2: 5
 * Encoder Pin Assignment:
 *   Channel B1: A2, Channel A1: A3
 */

#include <Arduino.h>
#include <Encoder.h>
#include "L298N.h"        
#include "DCMotorServo.h" 

// Define motor driver pins for L298N
#define L298_ENA 3
#define L298_IN1 4
#define L298_IN2 5

// Define encoder pins
#define ENCODER_PIN1 A2
#define ENCODER_PIN2 A3

// Create an instance of the L298N motor driver
L298N l298Motor(L298_ENA, L298_IN1, L298_IN2);

// Create an instance of the Encoder
Encoder myEncoder(ENCODER_PIN1, ENCODER_PIN2);

// Wrapper functions for the motor driver
void l298MotorWrite(int16_t speed)
{
    l298Motor.write(speed);
}

void l298MotorBrake()
{
    l298Motor.brake();
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
DCMotorServo servo(l298MotorWrite, l298MotorBrake, encoderReadFunc, encoderWriteFunc);

// PID parameters for tuning
float KP = 0.1;
float KI = 0.15;
float KD = 0.05;

void setup()
{
    Serial.begin(115200);

    // Initialize the motor driver
    l298Motor.begin();
    Serial.println("L298N motor initialized");

    // Configure the PID controller
    servo.myPID->SetTunings(KP, KI, KD);
    servo.setPWMSkip(50);
    servo.setAccuracy(14);

    Serial.println("DCMotorServo with L298N Example");
    Serial.println("Setpoint,Input,Output");
}

void loop()
{
    static unsigned long lastPrintTime = millis();

    // Run the PID loop
    servo.run();

    // Print PID data for Serial Plotter every 20ms
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

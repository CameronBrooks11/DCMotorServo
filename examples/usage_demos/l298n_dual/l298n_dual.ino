/*
 * Example: DCMotorServo with L298N driver
 *
 * L298N Motor Driver Pin Assignment:
 * - Motor1: ENA (PWM): 3, IN1: 4, IN2: 5
 * - Motor2: ENB (PWM): 6, IN3: 7, IN4: 8
 * Encoder Pin Assignment:
 * - Motor1: Channel B1: A2, Channel A1: A3
 * - Motor2: Channel B2: A6, Channel A1: A7
 */

#include <Arduino.h>
#include <Encoder.h>
#include "L298N.h"
#include "DCMotorServo.h"

// Define motor driver pins for L298N channel 1
#define L298_ENA 6
#define L298_IN1 8
#define L298_IN2 7
#define ENCODER_PINA1 2
#define ENCODER_PINA2 4

// Define motor driver pins for L298N channel 2
#define L298_ENB 9
#define L298_IN3 10
#define L298_IN4 11
#define ENCODER_PINB1 3
#define ENCODER_PINB2 5

// Create two instances of the L298N motor driver
L298N l298Motor1(L298_ENA, L298_IN1, L298_IN2);
L298N l298Motor2(L298_ENB, L298_IN3, L298_IN4);

// Create an instance of the Encoder for each motor
Encoder encoder1(ENCODER_PINA1, ENCODER_PINA2);
Encoder encoder2(ENCODER_PINB1, ENCODER_PINB2);

// Wrapper functions for the motor drivers
void l298Motor1Write(int16_t speed)
{
    l298Motor1.write(speed);
}
void l298Motor1Brake()
{
    l298Motor1.brake();
}
long encoder1ReadFunc()
{
    return encoder1.read();
}
void encoder1WriteFunc(long newPosition)
{
    encoder1.write(newPosition);
}
void l298Motor2Write(int16_t speed)
{
    l298Motor2.write(speed);
}
void l298Motor2Brake()
{
    l298Motor2.brake();
}
long encoder2ReadFunc()
{
    return encoder2.read();
}
void encoder2WriteFunc(long newPosition)
{
    encoder2.write(newPosition);
}

// Create an instance of DCMotorServo using the function pointers
DCMotorServo servo1(l298Motor1Write, l298Motor1Brake, encoder1ReadFunc, encoder1WriteFunc);
DCMotorServo servo2(l298Motor2Write, l298Motor2Brake, encoder2ReadFunc, encoder2WriteFunc);

// PID parameters for tuning
float KP1 = 0.1;
float KI1 = 0.15;
float KD1 = 0.05;

float KP2 = 0.1;
float KI2 = 0.15;
float KD2 = 0.05;

void setup()
{
    Serial.begin(115200);

    // Initialize the first motor driver
    l298Motor1.begin();
    Serial.println("L298N motor1 initialized");

    // Initialize the second motor driver
    l298Motor2.begin();
    Serial.println("L298N motor2 initialized");

    // Configure the PID controller for servo1
    servo1.myPID->SetTunings(KP1, KI1, KD1);
    servo1.setPWMSkip(50);
    servo1.setAccuracy(14);

    // Configure the PID controller for servo2
    servo2.myPID->SetTunings(KP2, KI2, KD2);
    servo2.setPWMSkip(50);
    servo2.setAccuracy(14);

    Serial.println("DCMotorServo with L298N Example");
    Serial.println("Setpoint,Input,Output");
}

void loop()
{
    static unsigned long lastPrintTime = millis();

    // Run the PID loop
    servo1.run();
    servo2.run();

    // Print PID data for Serial Plotter every 20ms
    if (millis() - lastPrintTime > 20)
    {
        lastPrintTime = millis();
        Serial.print(servo1.getSerialPlotter(1));
        Serial.print(", ");
        Serial.println(servo2.getSerialPlotter(2));
    }

    // Allow PID tuning commands via Serial
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("MOVE1="))
        {
            long newPosition = command.substring(6).toInt(); // Changed from substring(5) to substring(6)
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
            long newPosition = command.substring(6).toInt(); // Changed from substring(5) to substring(6)
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
            Serial.println("Invalid command, use MOVE1=<value> or MOVE2=<value>");
        }
    }
}

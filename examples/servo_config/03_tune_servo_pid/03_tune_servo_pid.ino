#include <Arduino.h>
#include <Encoder.h>
#include "DCMotorServo.h"

#define USE_LMD18500 1
// #define USE_L298N 1

// ----- LMD18500 Pin Definitions -----
#ifdef USE_LMD18500
#include "LMD18200.h"
#define LMD_PWM_PIN 6
#define LMD_DIR_PIN 7
#define LMD_BRAKE_PIN 8
#define ENCODER_PIN1 A2 // non-interrupt pins
#define ENCODER_PIN2 A3
#endif // USE_LMD18500

// ------- L298N Pin Definitions -------
#ifdef USE_L298N
#include "L298N.h"
#define L298_ENA 6
#define L298_IN1 7
#define L298_IN2 8
#define ENCODER_PIN1 2 // interrupt pins
#define ENCODER_PIN2 3
#endif // USE_L298N

// ----- Create Motor Instances -----
#ifdef USE_LMD18500
LMD18200 motorDriver(LMD_PWM_PIN, LMD_DIR_PIN, LMD_BRAKE_PIN);
#endif // USE_LMD18500

#ifdef USE_L298N
L298N motorDriver(L298_ENA, L298_IN1, L298_IN2);
#endif // USE_L298N

// ----- Create Encoder Instance -----
Encoder myEncoder(ENCODER_PIN1, ENCODER_PIN2);

// PID parameters for motor servo object
float KP = 0.1;
float KI = 0.15;
float KD = 0.05;

// Wrapper functions for the motor driver
void wrapMotorWrite(int16_t speed)
{
    motorDriver.write(speed);
}

void wrapMotorBrake()
{
    motorDriver.brake();
}

// Wrapper functions for the encoder
long wrapEncoderRead()
{
    return myEncoder.read();
}

void wrapEncoderWrite(long newPosition)
{
    myEncoder.write(newPosition);
}

// Create an instance of DCMotorServo using the function pointers
DCMotorServo servo(wrapMotorWrite, wrapMotorBrake, wrapEncoderRead, wrapEncoderWrite);

void setup()
{
    Serial.begin(115200);

    // Initialize the motor driver
    motorDriver.begin();

    // Configure the PID controller
    servo.myPID->SetTunings(KP, KI, KD);
    servo.setPWMSkip(35);
    servo.setAccuracy(8);
    servo.setMaxPWM(255);

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
            long newPosition = command.substring(5).toInt();
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

#include <Arduino.h>
#include <Encoder.h>
#include "DCMotorServo.h"
#include "DCMotorTacho.h"

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

#define SPEED_INTERVAL 20 // ms

// Create an instance of DCMotorTacho.
// For inner PID (speed control)
DCMotorTacho tacho(&servo, CPR, SPEED_INTERVAL);

// Outer loop PID tunings (position control) for the underlying servo, see servo_config for determining these.
float outerKp = 0.15, outerKi = 0.00, outerKd = 0.001;
// Inner loop PID tunings (speed control) will be adjusted via serial commands.
float innerKp = 0.6, innerKi = 0.2, innerKd = 0.01;

void setup()
{
    Serial.begin(115200);
    // Initialize the motor driver.
    motorDriver.begin();

    // Configure the outer (position) PID controller.
    servo.setPIDTunings(outerKp, outerKi, outerKd);
    servo.setPWMSkip(25);
    servo.setAccuracy(10);
    servo.setMaxPWM(255);

    tacho.setSpeedRPM(0);
    tacho.setPIDTunings(innerKp, innerKi, innerKd);

    Serial.println("DCMotorTacho (Cascaded Speed Control) Example");
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
            tacho.setPIDTunings(kp, innerKi, innerKd);
            innerKp = kp;
            Serial.print("Inner PID Kp set to: ");
            Serial.println(kp);
        }
        else if (command.startsWith("SPEEDKI="))
        {
            double ki = command.substring(8).toFloat();
            tacho.setPIDTunings(innerKp, ki, innerKd);
            innerKi = ki;
            Serial.print("Inner PID Ki set to: ");
            Serial.println(ki);
        }
        else if (command.startsWith("SPEEDKD="))
        {
            double kd = command.substring(8).toFloat();
            tacho.setPIDTunings(innerKp, innerKi, kd);
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
        } else if (command.startsWith("STOP"))
        {
            tacho.stop();
            Serial.println("Motor stopped.");
        }
        else
        {
            Serial.println("Invalid command. Commands: SPEED=, SPEEDKP=, SPEEDKI=, SPEEDKD=, KP=, KI=, KD=");
        }
    }
}

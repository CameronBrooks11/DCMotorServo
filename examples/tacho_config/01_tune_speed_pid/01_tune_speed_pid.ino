#include <Arduino.h>
#include "baseSpecs.h"
#include "DCMotorServo.h"
#include "DCMotorTacho.h"

// Create an instance of DCMotorServo using the shared wrappers.
DCMotorServo servo(wrapMotorWrite, wrapMotorBrake, wrapEncoderRead, wrapEncoderWrite);

// Create an instance of DCMotorTacho.
// For inner PID (speed control)
DCMotorTacho tacho(&servo, CPR, SPEED_INTERVAL);

// Outer loop PID tunings (position control) for the underlying servo, see servo_config for determining these.
float outerKp = TACHO_OUTER_KP, outerKi = TACHO_OUTER_KI, outerKd = TACHO_OUTER_KD;
// Inner loop PID tunings (speed control) will be adjusted via serial commands.
float innerKp = SPEED_KP, innerKi = SPEED_KI, innerKd = SPEED_KD;

void setup()
{
    Serial.begin(115200);
    // Initialize the motor driver.
    motorDriver.begin();

    // Configure the outer (position) PID controller.
    servo.setPIDTunings(outerKp, outerKi, outerKd);
    servo.setPWMSkip(PWM_SKIP);
    servo.setAccuracy(ACCURACY);
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

#include <Arduino.h>
#include "baseSpecs.h"
#include "DCMotorServo.h"

// PID parameters for motor servo object
float KP = SERVO_KP;
float KI = SERVO_KI;
float KD = SERVO_KD;

float fudgeFactor = EMPIRICAL_FUDGE_FACTOR; // Initial value for the empirical fudge factor
float CPRValue = CPR;                       // Initial value for the counts per revolution

// Create an instance of DCMotorServo using the function pointers
DCMotorServo servo(wrapMotorWrite, wrapMotorBrake, wrapEncoderRead, wrapEncoderWrite);

void setup()
{
    Serial.begin(115200);

    // Initialize the motor driver
    motorDriver.begin();

    // Configure the PID controller
    servo.myPID->SetTunings(KP, KI, KD);
    servo.setPWMSkip(PWM_SKIP);
    servo.setAccuracy(ACCURACY);
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
        else if (command.startsWith("FUDGE="))
        {
            float newFudgeFactor = command.substring(6).toFloat();
            if (newFudgeFactor > 0)
            {
                fudgeFactor = newFudgeFactor;
                Serial.print("Setting FUDGEFACTOR to: ");
                Serial.print(fudgeFactor);
                CPRValue = ENCODER_RESOLUTION * 4 * fudgeFactor;
                Serial.print(" and setting CPR to: ");
                Serial.println(CPRValue);
            }
            else
            {
                Serial.println("Invalid FUDGEFACTOR command");
            }
        }
        else if (command.startsWith("MOVEROTATE="))
        {
            long numRotations = command.substring(11).toInt();
            if (numRotations != 0)
            {
                servo.move(numRotations * CPRValue);
                Serial.print("Rotating ");
                Serial.print(numRotations);
                Serial.println(" full revolutions");
            }
            else
            {
                Serial.println("Invalid MOVEROTATE command");
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

/*
 * PWM Skip Tuning Procedure
 *
 * LMD18200 Motor Driver Pin Assignment:
 *   PWM: 6, DIR: 7, BRAKE: 8
 * Encoder Pin Assignment:
 *   Channel B1: A2, Channel A1: A3
 */

#include <Arduino.h>
#include <Encoder.h>
#include "LMD18200.h"

// ----- Pin Definitions -----
#define MOTOR_PWM_PIN 6
#define MOTOR_DIR_PIN 7
#define MOTOR_BRAKE_PIN 8
#define ENCODER_PIN1 A2
#define ENCODER_PIN2 A3

// ----- Tuning Constants -----
const unsigned long INIT_PWM_RAMP_INTERVAL = 400; // Initial interval (ms) between PWM increments
const int MOVE_STEPS_SENSITIVITY = 5;              // Minimum encoder steps to consider as movement
const int MAX_PWM = 255;                           // Maximum PWM value

// ----- Global Variables for Tuning -----
unsigned long pwmRampInterval; // Current PWM ramp interval
int currentPWM;                // Current PWM value being applied
int pwmStart;                  // Starting PWM for the current iteration
int pwmEstimate;               // Latest PWM estimate from encoder detection
int pwmEstimate0;              // PWM value recorded when movement is first detected

// Timers for independent branches
unsigned long lastPWMUpdateTime = 0;
unsigned long lastEncoderCheckTime = 0;

// ----- Create Instances -----
LMD18200 motorDriver(MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_BRAKE_PIN);
Encoder myEncoder(ENCODER_PIN1, ENCODER_PIN2);

void serialPrintResults()
{
    Serial.print("PWM_Start: ");
    Serial.print(pwmStart);
    Serial.print(", Current_PWM: ");
    Serial.print(currentPWM);
    Serial.print(", PWM_Estimate: ");
    Serial.print(pwmEstimate);
    Serial.print(", Ramp_Interval: ");
    Serial.print(pwmRampInterval);
    Serial.print(", Encoder: ");
    Serial.println(myEncoder.read());
}

void setup()
{
    Serial.begin(115200);
    motorDriver.begin();
    myEncoder.write(0); // Zero the encoder

    // Initialize tuning variables
    pwmRampInterval = INIT_PWM_RAMP_INTERVAL;
    pwmStart = currentPWM = 0;
    pwmEstimate = MAX_PWM;

    lastPWMUpdateTime = millis();
    lastEncoderCheckTime = millis();

    Serial.println("PWM Skip Tuning Procedure Start");
    Serial.println("PWM_Start, PWM_Estimate, Ramp_Interval");

    // Apply a brake and pause to allow the motor to settle.
    motorDriver.brake();
    delay(1000);
}

void loop()
{
    static bool tuningComplete = false;
    unsigned long currentTime = millis();

    // Terminate tuning once the estimate converges
    if (tuningComplete)
    {
        Serial.print("Tuning complete. Final pwmStart: ");
        Serial.print(pwmStart);
        Serial.print(", Final pwmEstimate: ");
        Serial.println(pwmEstimate);
        motorDriver.brake();
        while (1)
            ; // Halt execution
    }

    int encoderMovement = abs(myEncoder.read());
    if (encoderMovement >= MOVE_STEPS_SENSITIVITY)
    {
        // Significant movement detected.
        pwmEstimate0 = currentPWM;
        pwmEstimate = pwmEstimate0;

        // Update pwmStart to halfway between previous pwmStart and new estimate.
        pwmStart = pwmEstimate0 - ((pwmEstimate0 - pwmStart) / 2);

        // Double the ramp interval for the next iteration.
        pwmRampInterval *= 2;

        serialPrintResults();

        // Apply a brake and pause to allow the motor to settle.
        motorDriver.brake();
        delay(1000);

        // Reset encoder for the next iteration.
        myEncoder.write(0);
        // Reset current PWM to new start value.
        currentPWM = pwmStart;

        // Check for convergence.
        if (pwmEstimate == pwmStart)
        {
            tuningComplete = true;
        }
    }

    // ---- PWM Ramp Update Branch (every pwmRampInterval ms) ----
    else if (currentTime - lastPWMUpdateTime >= pwmRampInterval)
    {
        lastPWMUpdateTime = currentTime;
        motorDriver.write(currentPWM);

        // Only ramp up PWM if encoder movement is below sensitivity.
        if (abs(myEncoder.read()) < MOVE_STEPS_SENSITIVITY)
        {
            currentPWM++;
            if (currentPWM > MAX_PWM)
            {
                currentPWM = MAX_PWM;
            }
            serialPrintResults();
        }
    }
}

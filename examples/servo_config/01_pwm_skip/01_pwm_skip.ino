#include <Arduino.h>
#include <Encoder.h>

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

// ----- Tuning Constants -----
const unsigned long INIT_PWM_RAMP_INTERVAL = 400; // Initial interval (ms) between PWM increments
const int MOVE_STEPS_SENSITIVITY = 5;             // Minimum encoder steps to consider as movement
const int MAX_PWM = 255;                          // Maximum PWM value

// ----- Global Variables for Tuning -----
unsigned long pwmRampInterval; // Current PWM ramp interval
int currentPWM;                // Current PWM value being applied
int pwmStart;                  // Starting PWM for the current iteration
int pwmEstimate;               // Latest PWM estimate from encoder detection
int pwmEstimate0;              // PWM value recorded when movement is first detected

// Timers for independent branches
unsigned long lastPWMUpdateTime = 0;
unsigned long lastEncoderCheckTime = 0;

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

    long encoderMovement = abs(myEncoder.read());
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

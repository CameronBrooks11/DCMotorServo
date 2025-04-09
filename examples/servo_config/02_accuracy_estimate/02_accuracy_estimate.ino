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
const int MAX_PWM = 255;                         // Full speed
const int INIT_ACCURACY = 10;                    // Initial accuracy threshold (adjust as needed)
const int targetSteps = 5000;                    // Target steps for the motor to reach
const unsigned long STABILIZATION_PERIOD = 3000; // Time (ms) for stabilization control once target is passed
const int TRIALS_PER_ACCURACY = 5;               // Number of consecutive successful stabilization trials required per accuracy level
const unsigned long SYSTEM_DELAY = 2;            // Control update interval in ms to reflect the expected system delay

// ----- Global Variables for Tuning -----
int currentAccuracy; // Current accuracy threshold
long pos = 0;        // Variable to store encoder position

void serialPrintResults()
{
    Serial.print("Current Accuracy: ");
    Serial.print(currentAccuracy);
    Serial.print(", Target Steps: ");
    Serial.print(targetSteps);
    Serial.print(", Encoder Position: ");
    Serial.println(pos);
}

void setup()
{
    Serial.begin(115200);
    motorDriver.begin();
    myEncoder.write(0); // Zero the encoder

    // Initialize accuracy tuning parameters
    currentAccuracy = INIT_ACCURACY;

    Serial.println("Accuracy Estimate Tuning Procedure Start");
}

void loop()
{
    // Stabilization trials with bang-bang control.
    Serial.println("Starting stabilization trials...");
    int successCount = 0;
    int errorCount = 0;
    bool anyTrialSuccess = false; // Track if any trial succeeded

    for (int i = 0; i < TRIALS_PER_ACCURACY; i++)
    {
        // Reset encoder for this trial.
        myEncoder.write(0);

        // Run stabilization control for a fixed period.
        unsigned long stabilizeStart = millis();
        unsigned long lastSerialPrintTime = millis(); // For non-blocking Serial printing.
        unsigned long lastControlTime = millis();     // For SYSTEM_DELAY control updates.
        while (millis() - stabilizeStart < STABILIZATION_PERIOD)
        {
            pos = myEncoder.read();

            // Only update control every SYSTEM_DELAY ms.
            if (millis() - lastControlTime >= SYSTEM_DELAY)
            {
                lastControlTime = millis();
                if (pos < targetSteps - currentAccuracy)
                {
                    motorDriver.write(MAX_PWM);
                }
                else if (pos > targetSteps + currentAccuracy)
                {
                    motorDriver.write(-MAX_PWM);
                }
                else
                {
                    motorDriver.brake();
                }
            }

            // Only print results every 100 ms.
            if (millis() - lastSerialPrintTime >= 100)
            {
                serialPrintResults();
                lastSerialPrintTime = millis();
            }
        }

        motorDriver.write(0); // Release the motor and allow it to stop.

        // Report trial results.
        long finalError = abs(myEncoder.read() - targetSteps);
        if (finalError <= currentAccuracy)
        {
            successCount++;
            Serial.print("Trial successful. ");
            Serial.println(successCount);
        }
        else
        {
            errorCount++;
            Serial.print("Trial failed. ");
            Serial.println(finalError);
        }
        Serial.println("Successes: " + String(successCount) + ", Failures: " + String(errorCount));
    }

    // If all trials failed at this accuracy, end the procedure.
    if (errorCount >= successCount)

    {
        Serial.println("Stabilization failed at current accuracy.");
        Serial.print("Final achieved accuracy: ");
        Serial.println(currentAccuracy);
        motorDriver.write(0); // Release the motor and allow it to stop.
        while (1)
            ; // Halt execution.
    }

    // Refine accuracy threshold.
    int previousAccuracy = currentAccuracy;
    currentAccuracy = currentAccuracy - 1;
    if (currentAccuracy < 1)
    {
        currentAccuracy = 1;
    }

    Serial.print("Refining accuracy: previous = ");
    Serial.print(previousAccuracy);
    Serial.print(", new = ");
    Serial.println(currentAccuracy);

    myEncoder.write(0);
    delay(2000);

    if (previousAccuracy - currentAccuracy <= 0)
    {
        Serial.println("Final achieved accuracy:");
        Serial.println(previousAccuracy);
        motorDriver.brake();
        while (1)
            ; // Halt execution.
    }
}

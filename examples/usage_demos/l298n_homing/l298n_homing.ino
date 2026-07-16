/*
 * Example: DCMotorServo extrema sensing with an L298N driver
 *
 * Demonstrates the three extrema-sensing tiers:
 * - Physical: a limit switch (endstop) on the minimum side
 * - Electrical/sensorless: encoder-based stall detection
 * - Software: travel limits in encoder counts
 *
 * On startup the motor homes toward the minimum extreme. It stops when the
 * endstop triggers OR the mechanism stalls against a hard stop (whichever
 * happens first), zeroes the encoder there, and applies soft travel limits.
 *
 * L298N Motor Driver Pin Assignment:
 * - ENA (PWM): 6, IN1: 8, IN2: 7
 * Encoder Pin Assignment:
 * - Channel A: 2, Channel B: 4
 * Endstop Pin Assignment:
 * - Limit switch to GND on pin 12 (INPUT_PULLUP, LOW = triggered)
 */

#include <Arduino.h>
#include <Encoder.h>
#include "L298N.h"
#include "DCMotorServo.h"

#define L298_ENA 6
#define L298_IN1 8
#define L298_IN2 7
#define ENCODER_PINA 2
#define ENCODER_PINB 4
#define ENDSTOP_MIN_PIN 12

// Travel range in encoder counts, measured from the homed (min) position
#define TRAVEL_COUNTS 4000

L298N l298Motor(L298_ENA, L298_IN1, L298_IN2);
Encoder encoder(ENCODER_PINA, ENCODER_PINB);

// Wrapper functions for the motor driver and encoder
void motorWriteFunc(int16_t speed)
{
    l298Motor.write(speed);
}
void motorBrakeFunc()
{
    l298Motor.brake();
}
long encoderReadFunc()
{
    return encoder.read();
}
void encoderWriteFunc(long newPosition)
{
    encoder.write(newPosition);
}

// Endstop wrapper: switch closes to GND when triggered
bool endstopMinFunc()
{
    return digitalRead(ENDSTOP_MIN_PIN) == LOW;
}

DCMotorServo servo(motorWriteFunc, motorBrakeFunc, encoderReadFunc, encoderWriteFunc);

void setup()
{
    Serial.begin(115200);
    pinMode(ENDSTOP_MIN_PIN, INPUT_PULLUP);

    l298Motor.begin();

    servo.setPIDTunings(0.1, 0.15, 0.05);
    servo.setPWMSkip(50);
    servo.setAccuracy(14);

    // Physical extreme: limit switch on the minimum side only
    servo.attachEndstops(endstopMinFunc, nullptr);

    // Sensorless extreme: <4 counts of movement in 300 ms = stalled.
    // Doubles as a safety fault during normal moves.
    servo.enableStallDetection(300, 4);

    // Home toward the minimum extreme at PWM 120. Homing bypasses the software
    // travel limits; the third argument bounds the move as a failsafe in case
    // the endstop switch is dead and nothing ever stalls.
    if (servo.startHoming(-1, 120, TRAVEL_COUNTS + TRAVEL_COUNTS / 4))
    {
        Serial.println("Homing toward min extreme...");
    }
    else
    {
        Serial.println("Homing refused (no endstop or stall detection)");
    }
}

void loop()
{
    static bool homingReported = false;
    static bool stallReported = false;

    servo.run();

    // Wait for homing to finish, then check whether it actually succeeded
    if (!homingReported && !servo.isHoming())
    {
        homingReported = true;
        if (servo.isHomed())
        {
            servo.setTravelLimits(0, TRAVEL_COUNTS);
            Serial.print("Homed. Encoder zeroed, travel limits [0, ");
            Serial.print(TRAVEL_COUNTS);
            Serial.println("] set.");
            Serial.println("Send MOVETO=<counts> (out-of-range targets are clamped).");
        }
        else
        {
            Serial.println("Homing FAILED (cancelled or max_travel hit) - check the endstop.");
        }
    }

    // A stall during a normal move latches a fault: the motor stays braked.
    // Keep it latched until deliberate operator intervention (CLEAR command) -
    // auto-clearing would just slam the mechanism back into whatever jammed it.
    if (servo.isStalled() && !stallReported)
    {
        stallReported = true;
        Serial.println("STALL detected mid-move! Motor braked. Send CLEAR to recover.");
    }

    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.startsWith("MOVETO="))
        {
            long target = command.substring(7).toInt();
            servo.moveTo(target);
            Serial.print("Moving to: ");
            Serial.println(servo.getRequestedPosition()); // shows the clamped value
        }
        else if (command == "CLEAR")
        {
            servo.clearStall();
            servo.moveTo(0);
            stallReported = false;
            Serial.println("Stall cleared, returning to 0.");
        }
        else if (command == "HOME")
        {
            // Homing bypasses travel limits (they can stay set) and clears
            // any latched stall fault, so HOME also works as stall recovery
            servo.startHoming(-1, 120, TRAVEL_COUNTS + TRAVEL_COUNTS / 4);
            homingReported = false;
            stallReported = false;
            Serial.println("Re-homing...");
        }
        else
        {
            Serial.println("Commands: MOVETO=<counts> | CLEAR | HOME");
        }
    }
}

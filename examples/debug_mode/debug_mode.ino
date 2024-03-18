#include <Encoder.h>
#include <PID_v1.h>
#include <DCMotorServo.h>

#define pin_dcmoto_dir1 4
#define pin_dcmoto_dir2 5
#define pin_dcmoto_pwm_out 6
#define pin_dcmoto_encode1 2
#define pin_dcmoto_encode2 3

// Encoder specifications
#define PPR 11 // Pulses per revolution (PPR) of the encoder
#define GEAR_RATIO 29 // Gear ratio of the gearbox. Change this according to your motor's gear ratio.

// Calculate Counts Per Revolution (CPR) for quadrature encoders
#define CPR (PPR * 4 * GEAR_RATIO)

DCMotorServo servo = DCMotorServo(pin_dcmoto_dir1, pin_dcmoto_dir2, pin_dcmoto_pwm_out, pin_dcmoto_encode1, pin_dcmoto_encode2);

/**
 * PID Tuning Procedure:
 * 
 * The goal of PID tuning is to find the optimal set of parameters (Kp, Ki, Kd) that
 * minimizes error and stabilizes the response of the system as quickly as possible without
 * overshooting or oscillating.
 * 
 * 1. Start with setting Ki and Kd to 0, focus on tuning Kp first. Increase Kp until the system
 *    responds quickly to changes but without too much overshoot. If the system oscillates,
 *    Kp is too high.
 * 
 * 2. Once Kp is set, start increasing Ki until any steady-state error (the difference between
 *    the desired position and the actual position when the system is stable) is corrected
 *    quickly by the system. Be cautious, as too high a Ki can lead to instability and
 *    oscillation.
 * 
 * 3. Finally, adjust Kd to reduce overshoot and oscillation. Kd helps to predict system
 *    behavior and can dampen potential overshoot caused by Kp and Ki. However, too much Kd
 *    can make the system sluggish.
 * 
 * Note: This is a trial and error process. Start with small increments and test the system's
 *       response. Each system will have a unique set of optimal PID values due to differences
 *       in hardware and application requirements. Use serial commands to adjust and test PID
 *       values dynamically without re-uploading your sketch for faster tuning.
 * 
 * Additional Tips:
 * - Use a consistent testing procedure: make similar moves or rotations for each test.
 * - Observe both the system's immediate response and its longer-term stability.
 * - Document each change and its effect to understand the direction of your adjustments.
 * 
 * Example Applications:
 * - For a robotic arm, precise and stable movement to a position may prioritize minimal
 *   overshoot and steady-state error, thus requiring careful tuning of Ki and Kd.
 * - For a conveyor system, quicker response might be prioritized, potentially allowing for a
 *   higher Kp, assuming overshoot is less of a concern.
 */
 
// Define PID parameters as variables for easy adjustment
float KP = 0.1;
float KI = 0.15;
float KD = 0.05;

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  // Use defined PID parameters
  servo.myPID->SetTunings(KP, KI, KD);
  servo.setPWMSkip(50);
  servo.setAccuracy(14); // Accuracy based on encoder specifics
}


void loop() {
  static unsigned long debug_timeout = millis();
  
  servo.run();
  
  if (millis() - debug_timeout > 1000) { // Every 1000ms, print debug info
    debug_timeout = millis();
    Serial.println(servo.getDebugInfo());
  }

  // Check if data is available to read
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the command until a newline character
    command.trim(); // Remove any whitespace or newlines

    // Parse PID tuning commands
    if (command.startsWith("KP=")) {
      KP = command.substring(3).toFloat();
      servo.myPID->SetTunings(KP, KI, KD);
      Serial.print("KP set to: ");
      Serial.println(KP);
    }
    else if (command.startsWith("KI=")) {
      KI = command.substring(3).toFloat();
      servo.myPID->SetTunings(KP, KI, KD);
      Serial.print("KI set to: ");
      Serial.println(KI);
    }
    else if (command.startsWith("KD=")) {
      KD = command.substring(3).toFloat();
      servo.myPID->SetTunings(KP, KI, KD);
      Serial.print("KD set to: ");
      Serial.println(KD);
    }else if (command.startsWith("MOVE=")) {
      String value = command.substring(5); // Extract the command value
      if (value == "1R") {
        // If the command is to rotate one full revolution
        servo.move(CPR); // Move the motor by one full revolution based on CPR
        Serial.println("Rotating one full revolution");
      } else {
        int newPosition = value.toInt(); // Try to convert to integer for custom moves
        if (newPosition != 0) {
          servo.move(newPosition); // Move the motor to the new position
          Serial.print("Moving to: ");
          Serial.println(newPosition);
        } else {
          Serial.println("Invalid MOVE command");
        }
      }
    }
    else if (command.startsWith("MAXPWM=")) {
      int maxPWM = command.substring(7).toInt(); // Extract and convert max PWM value to integer
      if (maxPWM >= 0 && maxPWM <= 255) {
        servo.setMaxPWM(maxPWM); // Set the maximum PWM value
        Serial.print("Setting MAXPWM to: ");
        Serial.println(maxPWM);
      } else {
        Serial.println("Invalid MAXPWM command");
      }
    } else {
      Serial.println("Invalid command");
    }
  }
}

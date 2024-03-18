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

  // Send a label line to the Serial Plotter
  Serial.println("Setpoint,Input,Output"); // These labels correspond to value1, value2, value3 respectively
}

void loop() {
  static unsigned long lastPrintTime = millis();
  
  servo.run();
  
  // Print PID data for Serial Plotter every 20ms
  if (millis() - lastPrintTime > 20) {
    lastPrintTime = millis();
    Serial.println(servo.getSerialPlotter());
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
    }
    else if (command.startsWith("MOVE=")) {
      String value = command.substring(5); // Extract the command value
      if (value == "1R") {
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

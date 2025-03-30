# Docs

## Conceptual Planning

The goal is to refactor the DC motor control library to be **driver-agnostic**. Instead of hardcoding hardware-specific routines (e.g., direct pin manipulation via `analogWrite` or creating an `Encoder` object), the library will use function pointers for the motor and encoder interfaces. This will allow users to either write their own low-level routines or integrate an existing library, as long as the minimal required interfaces are provided.

### Key Concepts

- **DC Motor as an Actuator:**

  - A DC motor rotates in response to polarity and voltage.
  - It has two main states:
    - **Driving:** The motor is in active motion.
    - **Braking:** The motor is actively decelerated or held in position.
  - **Control Functions:**
    - `write(value)`: Applies a PWM (and direction) command to drive the motor.
    - `brake()`: Halts the motor.
      - Note that `brake()` is not the same as `write(0)` because braking may involve active braking rather than simply cutting power.

- **Relative Encoder as a Sensor:**

  - Tracks the rotational position of the motor shaft.
  - **Sensor Functions:**
    - `read()`: Returns the current encoder position.
    - `write(newPosition)`: Sets or resets the encoder's position (useful for zeroing or calibration).

- **Closed-Loop Control:**
  - The system uses a PID controller where:
    - The **plant** is the motor’s shaft.
    - The **feedback** is provided by the encoder.
    - The **reference** is the desired shaft position.

### Abstraction Strategy

The library will decouple hardware-specific logic from the control algorithms by requiring the user to supply function pointers that conform to the following interfaces:

- **Motor Driver Interface:**

  - **`MotorWriteFunc`**: A function pointer to write a PWM (and potentially direction) command.
  - **`MotorBrakeFunc`**: A function pointer to brake the motor (with braking if needed).

- **Encoder Interface:**
  - **`EncoderReadFunc`**: A function pointer to read the current encoder position.
  - **`EncoderWriteFunc`**: A function pointer to set/reset the encoder position.

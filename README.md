# Pico Manta Ray Wing Controller

## Overview

This project uses a Raspberry Pi Pico microcontroller running MicroPython to control a biomimetic manta ray wing mechanism. It utilises four 360-degree continuous rotation servos, each paired with a KY-040 rotary encoder for positional feedback. The control system employs independent PID controllers for each joint (Base (x2), Middle, Tip) to achieve a smooth, synchronised flapping motion based on sine wave patterns. Homing is performed using the push buttons on the rotary encoders, and the homing offsets are saved persistently to the Pico's flash memory.

## Hardware Requirements

* 1 x Raspberry Pi Pico (or Pico W)
* 4 x KY-040 Rotary Encoders (with integrated push buttons)
* 4 x 360-degree Continuous Rotation Servos
* 1 x PCA9685 16-Channel I2C PWM Servo Driver Board
* 1 x External Power Supply (5V-6V DC, capable of delivering sufficient current for all 4 servos under load - typically 3A minimum recommended)
* Jumper Wires
* USB Cable (for Pico programming/power during setup)
* (Optional) Breadboard
* The mechanical structure of the manta ray wing, linking servo outputs to encoders.

## Wiring

Ensure all components share a **common ground (GND)**. This includes the Pico, PCA9685, encoders, and the external servo power supply. Refer to the official Pico pinout diagram [cite: 206] for physical pin numbers.

| Component         | Signal         | Pico GPIO Pin | Pico Physical Pin | Notes                                      |
| :---------------- | :------------- | :------------ | :---------------- | :----------------------------------------- |
| **PCA9685** | SDA            | GP4           | 6                 | I2C0 Data                                  |
|                   | SCL            | GP5           | 7                 | I2C0 Clock                                 |
|                   | VCC (Logic)    | 3V3(OUT)      | 36                | Power for PCA chip (Stable 3.3V)           |
|                   | GND            | GND           | 38                | Common Ground                              |
|                   | **V+ (Servos)** | N/A          | N/A               | **Connect to External 5-6V Servo PSU +** |
|                   | **GND (Servos)**| N/A          | N/A               | **Connect to External 5-6V Servo PSU GND** |
| **Encoder 0** (Base 1) | CLK (A)      | GP14          | 19                | *(Verify Pin)* |
|                   | DT (B)         | GP15          | 20                | *(Verify Pin)* |
|                   | SW (Button)    | GP10          | 14                | *(Verify Pin)* Input with Pull-up          |
|                   | VCC (+)        | 3V3(OUT)      | 36                | Power for Encoder                          |
|                   | GND            | GND           | 38                | Common Ground                              |
| **Encoder 1** (Base 2) | CLK (A)      | GP16          | 21                | *(Verify Pin)* |
|                   | DT (B)         | GP17          | 22                | *(Verify Pin)* |
|                   | SW (Button)    | GP11          | 15                | *(Verify Pin)* Input with Pull-up          |
|                   | VCC (+)        | 3V3(OUT)      | 36                | Power for Encoder                          |
|                   | GND            | GND           | 38                | Common Ground                              |
| **Encoder 2** (Mid) | CLK (A)      | GP18          | 24                | *(Verify Pin)* |
|                   | DT (B)         | GP19          | 25                | *(Verify Pin)* |
|                   | SW (Button)    | GP12          | 16                | *(Verify Pin)* Input with Pull-up          |
|                   | VCC (+)        | 3V3(OUT)      | 36                | Power for Encoder                          |
|                   | GND            | GND           | 38                | Common Ground                              |
| **Encoder 3** (Tip) | CLK (A)      | GP20          | 26                | *(Verify Pin)* |
|                   | DT (B)         | GP21          | 27                | *(Verify Pin)* |
|                   | SW (Button)    | GP13          | 17                | *(Verify Pin)* Input with Pull-up          |
|                   | VCC (+)        | 3V3(OUT)      | 36                | Power for Encoder                          |
|                   | GND            | GND           | 38                | Common Ground                              |

**Note:** Double-check the specific GPIO pin numbers used in the `ENCODER_PINS` list within the `main.py` code and ensure your physical wiring matches exactly.

## Software Setup

1.  **MicroPython:** Install MicroPython firmware onto your Raspberry Pi Pico. Download the appropriate `.uf2` file from [micropython.org](https://micropython.org/download/RPI_PICO/) and flash it by holding the BOOTSEL button while connecting the Pico via USB, then copying the file to the `RPI-RP2` drive.
2.  **IDE:** Thonny IDE ([thonny.org](https://thonny.org/)) is recommended for ease of use with MicroPython on the Pico. Connect Thonny to the Pico by selecting the `MicroPython (Raspberry Pi Pico)` interpreter in `Tools -> Options -> Interpreter`.
3.  **Libraries:** Download the following MicroPython library files and upload them to the `/lib` folder on the Pico's filesystem using Thonny (`View -> Files -> Raspberry Pi Pico -> Right-click -> New directory` to create `lib` if needed):
    * `pca9685.py`: From [https://github.com/kevinmcaleer/pca9685_for_pico](https://github.com/kevinmcaleer/pca9685_for_pico) (or compatible version).
    * `rotary_irq_rp2.py`: From [https://github.com/miketeachman/micropython-rotary](https://github.com/miketeachman/micropython-rotary).
4.  **Main Script:** Copy the final version of the control code (including PID, homing, etc.) into Thonny and save it to the Pico's root directory as `main.py`. This ensures it runs automatically on startup.

## Code Overview

The `main.py` script performs the following:

* Imports necessary libraries (`machine`, `time`, `math`, `os`, `pca9685`, `rotary_irq_rp2`).
* Defines configuration constants for pins, servo channels, PID gains, and motion patterns.
* Includes helper functions (`us_to_duty`) and a PID controller class.
* Initializes hardware components (I2C, PCA9685, Encoders, Switches) and PID controllers.
* Implements startup logic:
    * Checks if any encoder button is pressed to force re-homing.
    * If no override, attempts to load previously saved homing offsets from `homing_offsets.txt`.
    * If loading fails or override was requested, runs the manual button-press homing sequence (`perform_homing`).
    * Saves the determined offsets back to `homing_offsets.txt` after manual homing.
* Enters the main control loop (`while True:`):
    * Calculates time delta (`dt`).
    * Generates target positions for each joint using sine waves based on time, frequency, amplitude, and phase offset.
    * Reads current positions from encoders (relative to homed offsets).
    * Computes PID output (effort) for each joint.
    * Converts effort to servo PWM duty cycle, clamping to limits.
    * Commands each servo via the PCA9685 board.
    * Includes optional periodic debug printing.
* Uses a `try...finally` block to ensure servos are released (PWM set to 0) upon exiting the script (e.g., via Ctrl+C in Thonny or an error).

## Usage

1.  **Save Code:** Ensure the main script is saved as `main.py` on the Pico and library files are in `/lib`.
2.  **Power On:** Connect the external servo power supply and power the Pico (e.g., via USB or an independent 5V supply connected to VBUS/VSYS and GND).
3.  **Homing Override (Optional):** To force re-homing, press and hold *any* encoder button *while* the Pico is powering up/resetting.
4.  **Homing Sequence:**
    * If *not* overriding and a valid `homing_offsets.txt` file exists, the script will load the saved offsets and proceed directly to the main loop.
    * If overriding, or if the file doesn't exist/is invalid, the script will print prompts asking you to move each joint to its zero position and press the corresponding encoder button once. Do this for all four joints.
    * Once all buttons are pressed, the new offsets will be saved, and the script will proceed.
5.  **Running:** The main control loop will start, driving the servos according to the motion pattern and PID control.
6.  **Stopping (if connected to Thonny):** Press `Ctrl+C` in the Thonny Shell. The `finally` block should execute, stopping the servos.
7.  **Stopping (Standalone):** Simply remove power. Homing will be required again on the next startup unless valid offsets were saved.

## Configuration & Tuning (CRITICAL!)

The following parameters in `main.py` **must be reviewed and tuned** for your specific hardware and desired motion:

* `ENCODER_PINS`: Verify the GPIO pin numbers match your physical wiring exactly.
* `SERVO_CHANNEL_MAP`: Ensure this list correctly maps your logical joints (0-3) to the physical channels (0-15) you plugged the servos into on the PCA9685 board.
* `STOP_VALUE_US`: Calibrate this value precisely for *your* 360 servos to find the exact pulse width (in microseconds) that makes them stop completely still.
* `MIN_PULSE_US` / `MAX_PULSE_US`: Adjust if your servos have a different operational range than 1000-2000µs.
* `PID_GAINS`: **This requires significant iterative tuning.** Start with low values (especially Ki and Kd). Tune Kp first for basic response, then Ki to eliminate steady-state error, then Kd to reduce overshoot/oscillation. Tune each joint independently if possible.
* `MOTION_FREQ_HZ`: Controls the overall speed of the flapping cycle.
* `MOTION_AMPLITUDE`: Controls how far (in encoder counts) each joint moves from its zero position. Needs tuning based on mechanical limits and desired look.
* `MOTION_PHASE_RAD`: Controls the timing delay between joints to create the wave effect. Needs tuning for the desired flapping style.

## Status

* Basic PID control structure implemented.
* Sine wave motion pattern generation included.
* Button-based homing with persistence and override functional.
* Requires significant tuning of PID and Motion parameters for optimal performance.

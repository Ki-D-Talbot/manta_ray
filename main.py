# main.py - Manta Ray Wing Control for Raspberry Pi Pico
# Controls 4x 360 servos with KY-040 encoder feedback via PCA9685.
# Implements PID control, sine wave motion pattern, persistent button homing with override.

import machine
import time
import math
import os # Needed for statvfs and file operations

# --- Library Imports ---
# Ensure pca9685.py and rotary_irq_rp2.py are in the /lib folder on the Pico
try:
    from pca9685 import PCA9685
    from rotary_irq_rp2 import RotaryIRQ
except ImportError as e:
    print("----------------------------------------------------")
    print(f"ERROR: Ensure library files are in /lib. {e}")
    print("Needed: pca9685.py, rotary_irq_rp2.py")
    print("----------------------------------------------------")
    # Flash onboard LED rapidly to indicate fatal error
    led = machine.Pin("LED", machine.Pin.OUT) # Pico W LED
    # led = machine.Pin(25, machine.Pin.OUT) # Standard Pico LED
    while True: led.toggle(); time.sleep_ms(100)
    # Code will likely halt here anyway due to the error when run automatically

# --- Configuration ---
# I2C Pins (Using I2C0)
I2C_SCL_PIN = 5 # Pico Pin 7 (GP5)
I2C_SDA_PIN = 4 # Pico Pin 6 (GP4)
I2C_BUS_ID = 0

# Encoder Pins [[CLK, DT, SW], ...] - !!! VERIFY THESE GPIO NUMBERS MATCH WIRING !!!
ENCODER_PINS = [
    [14, 15, 10], # Base 1 (Joint 0) CLK, DT, SW -> GP14, GP15, GP10
    [16, 17, 11], # Base 2 (Joint 1) CLK, DT, SW -> GP16, GP17, GP11
    [18, 19, 12], # Mid    (Joint 2) CLK, DT, SW -> GP18, GP19, GP12
    [20, 21, 13]  # Tip    (Joint 3) CLK, DT, SW -> GP20, GP21, GP13
]
NUM_CONTROLLERS = len(ENCODER_PINS) # Should be 4

# PCA9685 Servo Channel Mapping (Connect servos accordingly)
# Map Joint index (0-3) to PCA channel (0-15)
SERVO_CHANNEL_MAP = [0, 1, 2, 3] # e.g., Joint0 -> Ch0, Joint1 -> Ch1, etc.

# Servo Parameters
SERVO_FREQ_HZ = 50   # Standard servo frequency
# !!! USE YOUR PREVIOUSLY CALIBRATED STOP VALUE HERE !!!
STOP_VALUE_US = 1500 # Pulse width in microseconds for servo stop
MIN_PULSE_US = 1000  # Lower PWM limit (microseconds) - safety boundary
MAX_PULSE_US = 2000  # Upper PWM limit (microseconds) - safety boundary

# PID Gains [ (Kp, Ki, Kd), ... ] for each joint - !!! THESE REQUIRE CAREFUL TUNING !!!
PID_GAINS = [
    (0.8, 0.1, 0.05), # Joint 0: Base 1 - Start tuning Kp first
    (0.8, 0.1, 0.05), # Joint 1: Base 2 - Tune independently even if target is sync'd
    (0.6, 0.1, 0.04), # Joint 2: Mid
    (0.5, 0.05, 0.03)  # Joint 3: Tip
]
# Anti-windup: Prevent integral term from growing too large
# Tune this based on Kp/Ki and typical error magnitudes if needed
MAX_INTEGRAL = 500.0

# Motion Pattern Parameters - !!! TUNE THESE FOR DESIRED FLAPPING MOTION !!!
MOTION_FREQ_HZ = 0.4 # Flapping speed (cycles per second) - Start slow (e.g., 0.2-0.5)
# Amplitude in encoder counts (+/- range from zero position) for each joint
MOTION_AMPLITUDE = [400, 400, 300, 200] # Base1, Base2, Mid, Tip - Start smaller
# Phase offset in radians (creates the wave delay down the wing)
MOTION_PHASE_RAD = [0, 0, math.pi / 3, math.pi / 1.5] # Base sync'd, Mid delayed, Tip delayed more

# File for saving/loading homing offsets
HOMING_FILE = "homing_offsets.txt"

# Debug Printing Control
ENABLE_DEBUG_PRINT = True # Set to False to disable periodic printing
DEBUG_PRINT_INTERVAL_MS = 500 # How often to print debug info (milliseconds)

# --- Helper Functions ---
us_per_cycle = 1_000_000 / SERVO_FREQ_HZ
def us_to_duty(microseconds):
    """Converts pulse width in microseconds to PCA9685 duty cycle (0-4095)"""
    duty = int((microseconds / us_per_cycle) * 4095)
    # Clamp duty cycle to the valid 12-bit range
    return max(0, min(duty, 4095))

# --- PID Controller Class ---
class PID:
    """A simple PID controller class."""
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0.0, max_integral=500.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.max_integral = abs(max_integral) # Anti-windup limit
        self._integral = 0.0
        self._last_error = 0.0
        print(f"PID initialized with Kp={Kp}, Ki={Ki}, Kd={Kd}, MaxInt={max_integral}")

    def compute(self, measured_value, dt):
        """Calculate PID output."""
        if dt <= 0: return 0 # Prevent division by zero if dt is invalid

        error = self.setpoint - measured_value

        # Integral term with anti-windup clamping
        self._integral += error * dt
        self._integral = max(-self.max_integral, min(self._integral, self.max_integral))

        # Derivative term (on error change)
        derivative = (error - self._last_error) / dt

        # Update last error *after* calculating derivative
        self._last_error = error

        # Calculate PID output
        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        return output

    def set_setpoint(self, setpoint):
        """Update the target setpoint."""
        self.setpoint = setpoint
        # Optional: Reset integral & last_error when setpoint changes?
        # Could be useful if motion pattern changes drastically.
        # self._integral = 0.0
        # self._last_error = 0.0

    def reset_state(self):
        """Reset integral and last error."""
        self._integral = 0.0
        self._last_error = 0.0

# --- Global Variables ---
pca = None
encoders = []
pids = []
encoder_switches = []
encoder_offsets = [0] * NUM_CONTROLLERS
homed_status = [False] * NUM_CONTROLLERS
is_homed = False # Overall homing complete flag

# --- Initialization Function ---
def initialize_hardware():
    """Initialize I2C, PCA9685, Encoders, Switches, and PID controllers."""
    global pca, encoders, pids, encoder_switches
    try:
        print("Initializing I2C...")
        i2c = machine.I2C(I2C_BUS_ID, scl=machine.Pin(I2C_SCL_PIN), sda=machine.Pin(I2C_SDA_PIN))
        print("Initializing PCA9685...")
        pca = PCA9685(i2c)
        pca.freq(SERVO_FREQ_HZ)
        print(f"PCA9685 frequency set to {SERVO_FREQ_HZ} Hz.")

        print("Initializing Encoders & Switches...")
        for i in range(NUM_CONTROLLERS):
            encoders.append(RotaryIRQ(pin_num_clk=ENCODER_PINS[i][0],
                                      pin_num_dt=ENCODER_PINS[i][1],
                                      pull_up=True))
            encoder_switches.append(machine.Pin(ENCODER_PINS[i][2], machine.Pin.IN, machine.Pin.PULL_UP))
            print(f" Joint {i}: Enc(GP{ENCODER_PINS[i][0]}, GP{ENCODER_PINS[i][1]}), SW(GP{ENCODER_PINS[i][2]})")

        print("Initializing PID Controllers...")
        for i in range(NUM_CONTROLLERS):
            pids.append(PID(Kp=PID_GAINS[i][0], Ki=PID_GAINS[i][1], Kd=PID_GAINS[i][2], max_integral=MAX_INTEGRAL))

        print("Initialization Complete.")
        return True # Indicate success

    except Exception as e:
        print(f"ERROR during initialization: {e}")
        return False # Indicate failure

# --- Homing Function (Includes Saving) ---
def perform_homing():
    """ Waits for each encoder button to be pressed to set the zero offset,
        then saves the offsets to a file. """
    global is_homed, encoder_offsets, homed_status
    print("\n--- Starting Homing Sequence ---")
    print("Manually move each joint to its desired ZERO position.")
    print("Press the corresponding encoder button ONCE when ready.")
    print("Waiting for buttons...")

    homed_status = [False] * NUM_CONTROLLERS # Reset status for this homing session
    all_homed = False
    while not all_homed:
        all_homed = True # Assume done unless proven otherwise
        for i in range(NUM_CONTROLLERS):
            if not homed_status[i]: # If this joint not yet homed
                all_homed = False # We are not done yet
                # Check if button is pressed (value() == 0 for pull-up)
                if encoder_switches[i].value() == 0:
                    time.sleep_ms(20) # Debounce delay
                    if encoder_switches[i].value() == 0: # Check again after delay
                        encoder_offsets[i] = encoders[i].value() # Set current value as offset
                        homed_status[i] = True # Mark as homed
                        pids[i].set_setpoint(0.0) # Set initial target to 0
                        pids[i].reset_state()     # Reset PID state
                        print(f"*** Joint {i} HOMED at offset: {encoder_offsets[i]} ***")
                        # Prevent holding button causing rapid re-triggering immediately
                        while encoder_switches[i].value() == 0:
                            time.sleep_ms(10)
        time.sleep_ms(50) # Polling delay

    # --- SAVE offsets to file ---
    try:
        print(f"Saving homing offsets to {HOMING_FILE}...")
        with open(HOMING_FILE, "w") as f:
            # Convert list of numbers to a comma-separated string
            offset_str = ",".join(map(str, encoder_offsets))
            f.write(offset_str)
        print("Offsets saved successfully.")
    except Exception as e:
        print(f"ERROR: Failed to save homing offsets: {e}")
        # Homing completed but won't persist

    is_homed = True
    print("--- All Joints Homed ---")

# --- Attempt to Load Homing Data ---
def load_homing_data():
    """ Loads homing data from file if it exists and is valid.
        Returns True if loaded successfully, False otherwise. """
    global encoder_offsets, is_homed, homed_status
    try:
        print(f"Attempting to load homing data from {HOMING_FILE}...")
        with open(HOMING_FILE, "r") as f:
            offset_str = f.read()
        # Convert comma-separated string back to list of integers
        loaded_offsets = list(map(int, offset_str.split(',')))
        # Verify data looks correct (correct number of entries)
        if len(loaded_offsets) == NUM_CONTROLLERS:
            encoder_offsets = loaded_offsets
            is_homed = True # Mark as homed since we loaded data
            homed_status = [True] * NUM_CONTROLLERS # Also update status list
            print("Homing data loaded successfully:")
            for i in range(NUM_CONTROLLERS):
                 print(f" Joint {i} offset: {encoder_offsets[i]}")
                 # Initialize PID controllers assuming loaded home is position 0
                 pids[i].set_setpoint(0.0)
                 pids[i].reset_state()
            return True # Load successful
        else:
            print(f"ERROR: Invalid data length in {HOMING_FILE} ({len(loaded_offsets)} entries, expected {NUM_CONTROLLERS}).")
            return False # Load failed
    except OSError:
        # File likely doesn't exist (e.g., first run)
        print(f"{HOMING_FILE} not found.")
        return False # Load failed
    except Exception as e:
        print(f"ERROR loading homing data: {e}")
        return False # Load failed

# --- Check for Homing Override at Startup ---
def check_homing_override():
    """ Checks if any encoder button is pressed during startup. """
    print("Checking for homing override request (press any encoder button)...")
    # Check immediately after power-on / script start
    # A short delay allows user time to press during boot
    time.sleep(0.5)
    for i in range(NUM_CONTROLLERS):
        if encoder_switches[i].value() == 0: # Button pressed (active low)
            print("!!! Homing override button pressed! Forcing manual homing. !!!")
            # Optionally delete the old homing file if override detected
            try: os.remove(HOMING_FILE)
            except OSError: pass # Ignore if file didn't exist
            return True # Override requested
    print("No override request detected.")
    return False # No override requested

# --- Main Program Execution ---
if initialize_hardware():
    force_homing = check_homing_override()

    if not force_homing:
        load_successful = load_homing_data()
        if not load_successful:
            print("Proceeding to manual homing sequence (Load Failed).")
            perform_homing()
    else:
        # Override was requested
        print("Proceeding to manual homing sequence (Override Request).")
        perform_homing()

    # Final check if homing actually completed
    if not is_homed:
        print("FATAL ERROR: Homing did not complete successfully. Halting.")
        # Add visual error signal here if desired
        while True: time.sleep(1)

    # --- Main Control Loop ---
    start_time_ms = time.ticks_ms()
    last_time_us = time.ticks_us()
    last_print_ms = start_time_ms

    print("\nStarting Main Control Loop - Flapping Motion...")
    print("Ensure Servo Power is ON. Press Ctrl+C in Thonny Shell to stop.")

    try:
        while True:
            # Wrap inner loop logic in try/except to catch runtime errors
            try:
                # --- Timing Calculation ---
                current_time_us = time.ticks_us()
                dt_us = time.ticks_diff(current_time_us, last_time_us)
                if dt_us <= 0 or dt_us > 500000: dt = 0.02 # Default if weird dt
                else: dt = dt_us / 1_000_000.0
                last_time_us = current_time_us

                # --- Motion Pattern Generation ---
                elapsed_time_s = time.ticks_diff(time.ticks_ms(), start_time_ms) / 1000.0
                base_angle = 2.0 * math.pi * MOTION_FREQ_HZ * elapsed_time_s
                target_positions = [0.0] * NUM_CONTROLLERS
                for i in range(NUM_CONTROLLERS):
                    target = MOTION_AMPLITUDE[i] * math.sin(base_angle + MOTION_PHASE_RAD[i])
                    target_positions[i] = target
                    pids[i].set_setpoint(target)

                # --- PID Control Loops ---
                for i in range(NUM_CONTROLLERS):
                    current_position = encoders[i].value() - encoder_offsets[i]
                    effort = pids[i].compute(current_position, dt)
                    output_us = STOP_VALUE_US + int(effort) # Scale 'effort' if needed
                    output_us = max(MIN_PULSE_US, min(output_us, MAX_PULSE_US)) # Clamp
                    duty = us_to_duty(output_us)
                    servo_channel = SERVO_CHANNEL_MAP[i]
                    pca.duty(servo_channel, duty)

                # --- Optional Debug Print ---
                if ENABLE_DEBUG_PRINT:
                    current_time_ms = time.ticks_ms()
                    if time.ticks_diff(current_time_ms, last_print_ms) >= DEBUG_PRINT_INTERVAL_MS:
                        # Print info for Joint 0 (Base 1) and Joint 2 (Mid) as examples
                        j0_cur = encoders[0].value()-encoder_offsets[0]
                        j2_cur = encoders[2].value()-encoder_offsets[2]
                        print(f"T:{elapsed_time_s:5.1f} | J0 Tgt:{target_positions[0]:<5.0f} Cur:{j0_cur:<5} | J2 Tgt:{target_positions[2]:<5.0f} Cur:{j2_cur:<5}")
                        last_print_ms = current_time_ms

                # --- Loop Delay ---
                time.sleep_ms(10) # Adjust for desired control loop frequency (~50-100Hz is often good)

            except KeyboardInterrupt: # Catch Ctrl+C specifically to break loop
                print("\nCtrl+C detected, stopping loop.")
                break
            except Exception as e: # Catch other unexpected errors in the loop
                print(f"\nERROR in main loop: {e}")
                # import sys # Uncomment these lines for more detailed tracebacks
                # sys.print_exception(e)
                print("Attempting to stop servos and exit.")
                break # Exit the while loop

    finally:
        # --- Cleanup ---
        print("\nExecuting finally block: Releasing servos.")
        if 'pca' in locals() and pca is not None: # Check if pca exists
            for i in range(NUM_CONTROLLERS):
                try:
                    # Setting duty to 0 should disable PWM output on most PCA9685 setups
                    pca.duty(SERVO_CHANNEL_MAP[i], 0)
                    print(f"  Servo on channel {SERVO_CHANNEL_MAP[i]} released.")
                except Exception as e:
                    print(f"  Error releasing servo on channel {SERVO_CHANNEL_MAP[i]}: {e}")
        else:
            print("  PCA object was not initialized, cannot release servos.")
        print("Program cleanup finished.")

else: # If initialize_hardware() returned False
    print("Hardware initialization failed. Program cannot continue.")
    # Add visual error signal here if desired
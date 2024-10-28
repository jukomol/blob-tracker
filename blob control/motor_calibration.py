import time
import json
import pigpio  # Library for GPIO PWM control on the Raspberry Pi

# GPIO pin configuration for ESC control
MOTOR_PIN_1 = 12
MOTOR_PIN_2 = 13

# Define full and minimum throttle PWM values for calibration
FULL_THROTTLE_PWM = 2000  # Adjust if needed (max PWM for full throttle)
MIN_THROTTLE_PWM = 1000   # Adjust if needed (min PWM for zero throttle)

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon.")
    exit()

def calibrate_motor(motor_pin):
    """
    Calibrate a single ESC connected to the given GPIO pin.
    """
    print(f"Starting ESC calibration on GPIO {motor_pin}. Please ensure the motor is not connected to a propeller.")

    # Step 1: Set full throttle
    pi.set_servo_pulsewidth(motor_pin, FULL_THROTTLE_PWM)
    print(f"[GPIO {motor_pin}] Set full throttle PWM to {FULL_THROTTLE_PWM}. Waiting for ESC to enter calibration mode...")
    time.sleep(2)  # Wait for ESC to recognize full throttle signal

    # Step 2: Set minimum throttle
    pi.set_servo_pulsewidth(motor_pin, MIN_THROTTLE_PWM)
    print(f"[GPIO {motor_pin}] Set minimum throttle PWM to {MIN_THROTTLE_PWM}. Finalizing calibration...")
    time.sleep(2)  # Wait for ESC to recognize minimum throttle signal

    # Step 3: Turn off signal to exit calibration mode
    pi.set_servo_pulsewidth(motor_pin, 0)
    time.sleep(1)

    print(f"[GPIO {motor_pin}] ESC calibration complete.")
    return {"min_pwm": MIN_THROTTLE_PWM, "max_pwm": FULL_THROTTLE_PWM}

def save_calibration(calibration_data):
    """
    Save calibration data to a JSON file.
    """
    with open("motor_calibration.json", "w") as f:
        json.dump(calibration_data, f, indent=4)
    print("Calibration data saved to motor_calibration.json")

def load_calibration():
    """
    Load calibration data from file.
    """
    try:
        with open("motor_calibration.json", "r") as f:
            calibration_data = json.load(f)
            print("Calibration data loaded:", calibration_data)
            return calibration_data
    except FileNotFoundError:
        print("Calibration file not found. Please calibrate the ESCs first.")
        return None

if __name__ == "__main__":
    # Load existing calibration data if available
    calibration_data = load_calibration()
    if calibration_data is None:
        # Calibrate MOTOR_PIN_1
        calibration_data = {}
        calibration_data["motor_1"] = calibrate_motor(MOTOR_PIN_1)

        # Calibrate MOTOR_PIN_2
        calibration_data["motor_2"] = calibrate_motor(MOTOR_PIN_2)

        # Save calibration data
        save_calibration(calibration_data)
    else:
        print("ESCs are already calibrated. No need to recalibrate.")
    
    # Stop PWM signal and cleanup
    pi.set_servo_pulsewidth(MOTOR_PIN_1, 0)
    pi.set_servo_pulsewidth(MOTOR_PIN_2, 0)
    pi.stop()

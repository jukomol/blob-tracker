import time
import json
import RPi.GPIO as GPIO

# GPIO pin configuration for ESC control
MOTOR_PIN_1 = 12
MOTOR_PIN_2 = 13

# Define full and minimum throttle PWM values for calibration
FULL_THROTTLE_PWM = 2000  # This corresponds to full throttle
MIN_THROTTLE_PWM = 1000   # This corresponds to minimum throttle

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN_1, GPIO.OUT)
GPIO.setup(MOTOR_PIN_2, GPIO.OUT)

# Initialize PWM with 50Hz frequency (common for ESCs)
pwm_motor_1 = GPIO.PWM(MOTOR_PIN_1, 50)  # 50Hz frequency
pwm_motor_2 = GPIO.PWM(MOTOR_PIN_2, 50)  # 50Hz frequency

# Start PWM with 0 duty cycle (motor off)
pwm_motor_1.start(0)
pwm_motor_2.start(0)

def pwm_from_microseconds(microseconds):
    """
    Convert microsecond PWM values (1000-2000 µs) to RPi.GPIO duty cycle percentage.
    """
    return (microseconds / 20000) * 100  # Convert to duty cycle for 50Hz

def calibrate_motor(motor_pwm, full_throttle, min_throttle):
    """
    Calibrate a single ESC connected to the given GPIO pin.
    """
    print("Starting ESC calibration. Please ensure the motor is not connected to a propeller.")

    # Step 1: Set full throttle
    motor_pwm.ChangeDutyCycle(pwm_from_microseconds(full_throttle))
    print(f"Set full throttle PWM to {full_throttle} µs. Waiting for ESC to enter calibration mode...")
    time.sleep(2)  # Wait for ESC to recognize full throttle signal

    # Step 2: Set minimum throttle
    motor_pwm.ChangeDutyCycle(pwm_from_microseconds(min_throttle))
    print(f"Set minimum throttle PWM to {min_throttle} µs. Finalizing calibration...")
    time.sleep(2)  # Wait for ESC to recognize minimum throttle signal

    # Step 3: Turn off signal to exit calibration mode
    motor_pwm.ChangeDutyCycle(0)
    time.sleep(1)

    print("ESC calibration complete.")
    return {"min_pwm": min_throttle, "max_pwm": full_throttle}

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
        calibration_data["motor_1"] = calibrate_motor(pwm_motor_1, FULL_THROTTLE_PWM, MIN_THROTTLE_PWM)

        # Calibrate MOTOR_PIN_2
        calibration_data["motor_2"] = calibrate_motor(pwm_motor_2, FULL_THROTTLE_PWM, MIN_THROTTLE_PWM)

        # Save calibration data
        save_calibration(calibration_data)
    else:
        print("ESCs are already calibrated. No need to recalibrate.")
    
    # Stop PWM signals and cleanup
    pwm_motor_1.stop()
    pwm_motor_2.stop()
    GPIO.cleanup()

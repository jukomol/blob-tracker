import RPi.GPIO as GPIO
import time

# GPIO pin configuration for ESC control
MOTOR_PIN_1 = 12  # Motor 1 ESC PWM pin
MOTOR_PIN_2 = 13  # Motor 2 ESC PWM pin
PWM_FREQUENCY = 50  # ESCs typically use 50Hz PWM

# Throttle values for ESC calibration
MAX_THROTTLE = 12.5  # Adjust these values as per your ESCs' requirements
MIN_THROTTLE = 7.5
MEDIUM_THROTTLE = 10  # Medium throttle for the test run

def initialize_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOTOR_PIN_1, GPIO.OUT)
    GPIO.setup(MOTOR_PIN_2, GPIO.OUT)

    # Create PWM instances on each motor pin
    pwm_motor1 = GPIO.PWM(MOTOR_PIN_1, PWM_FREQUENCY)
    pwm_motor2 = GPIO.PWM(MOTOR_PIN_2, PWM_FREQUENCY)
    
    # Start PWM with 0 duty cycle (off)
    pwm_motor1.start(0)
    pwm_motor2.start(0)
    
    return pwm_motor1, pwm_motor2

def calibrate_esc(pwm_motor):
    print("Calibrating ESC...")
    pwm_motor.ChangeDutyCycle(MAX_THROTTLE)
    time.sleep(2)  # Hold max throttle for 2 seconds
    pwm_motor.ChangeDutyCycle(MIN_THROTTLE)
    time.sleep(2)  # Hold min throttle for 2 seconds
    print("Calibration complete.")

def run_motor_test(pwm_motor1, pwm_motor2):
    # Set both motors to a medium throttle
    pwm_motor1.ChangeDutyCycle(MEDIUM_THROTTLE)
    pwm_motor2.ChangeDutyCycle(MEDIUM_THROTTLE)
    print("Running motors for 10 seconds at medium throttle...")
    time.sleep(10)

    # Stop motors after 10 seconds
    pwm_motor1.ChangeDutyCycle(0)
    pwm_motor2.ChangeDutyCycle(0)
    print("Motor test complete.")

def cleanup_gpio():
    GPIO.cleanup()
    print("GPIO cleanup complete.")

# Main script
if __name__ == "__main__":
    try:
        # Initialize GPIO and PWM
        pwm_motor1, pwm_motor2 = initialize_gpio()
        
        # Calibrate both ESCs
        calibrate_esc(pwm_motor1)
        calibrate_esc(pwm_motor2)
        
        # Run motors for a 10-second test
        run_motor_test(pwm_motor1, pwm_motor2)
        
    except KeyboardInterrupt:
        print("Process interrupted by user.")
    
    finally:
        # Cleanup GPIO resources
        cleanup_gpio()

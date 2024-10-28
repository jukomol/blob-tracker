from flask import Flask, render_template, Response, request
import cv2
import numpy as np
import json
import time
import RPi.GPIO as GPIO
import config
from blob_detection import capture_blob_error, get_mask  # Ensure get_mask function is available
from pid_controller import PIDController
from imu_integration import get_swing_correction

# GPIO pin configuration for ESC control
MOTOR_PIN_1 = 12
MOTOR_PIN_2 = 13

# Initialize Flask app
app = Flask(__name__)

# Initialize the video capture with the USB camera
camera = cv2.VideoCapture(1)  # Change to 0 for the default camera

# Initialize PID controller for x and y axes
pid = PIDController(config.Kp, config.Ki, config.Kd, config.dt)

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN_1, GPIO.OUT)
GPIO.setup(MOTOR_PIN_2, GPIO.OUT)

# Initialize PWM for motors at 50Hz frequency
pwm_motor_1 = GPIO.PWM(MOTOR_PIN_1, 50)  # 50Hz frequency for ESC
pwm_motor_2 = GPIO.PWM(MOTOR_PIN_2, 50)  # 50Hz frequency for ESC
pwm_motor_1.start(0)  # Start with motor off
pwm_motor_2.start(0)  # Start with motor off

# Load motor calibration data
with open("motor_calibration.json", "r") as f:
    calibration_data = json.load(f)
motor_1_cal = calibration_data["motor_1"]
motor_2_cal = calibration_data["motor_2"]

# Function to convert microsecond PWM values to duty cycle for 50Hz PWM
def pwm_from_microseconds(microseconds):
    return (microseconds / 20000) * 100  # Converts to duty cycle for 50Hz

# Motor control function
def set_motor_speed(motor_pwm, speed_pwm, motor_cal):
    """
    Set motor speed with PWM, constrained by calibrated min and max PWM values.
    """
    pwm_value = max(motor_cal["min_pwm"], min(motor_cal["max_pwm"], speed_pwm))
    motor_pwm.ChangeDutyCycle(pwm_from_microseconds(pwm_value))

def generate_frames():
    """
    Capture frames from the camera, perform blob detection, calculate errors,
    apply PID control, and send the frame with overlay in real-time.
    """
    while True:
        # Capture frame and calculate blob error
        error_x, error_y, frame = capture_blob_error(camera)
        
        # Generate the binary mask for HSV tuning
        mask = get_mask(frame)
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # Convert to BGR to display alongside the frame

        # If blob detected, calculate PID outputs
        if error_x is not None and error_y is not None:
            # Calculate PID outputs for x and y axes
            output_x, output_y = pid.compute(error_x, error_y)
            
            # Apply swing correction based on IMU
            swing_correction_x, swing_correction_y, _ = get_swing_correction(swing_gain=0.1)
            output_x += swing_correction_x
            output_y += swing_correction_y

            # Map PID outputs to motor PWM values
            motor_speed_1 = 1500 + output_x - output_y
            motor_speed_2 = 1500 - output_x + output_y

            # Set motor speeds, ensuring values are within the calibrated range
            set_motor_speed(pwm_motor_1, motor_speed_1, motor_1_cal)
            set_motor_speed(pwm_motor_2, motor_speed_2, motor_2_cal)

            # Display the error, motor speeds, and IMU corrections on the frame
            overlay_text = f"Error X: {error_x}, Error Y: {error_y}"
            cv2.putText(frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            motor_text = f"Motor 1 PWM: {int(motor_speed_1)}, Motor 2 PWM: {int(motor_speed_2)}"
            imu_text = f"Swing Correction X: {swing_correction_x:.2f}, Y: {swing_correction_y:.2f}"
            cv2.putText(frame, motor_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(frame, imu_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Combine the original frame with the mask for side-by-side view
        combined_frame = np.hstack((frame, mask_colored))

        # Show frame with overlay
        ret, buffer = cv2.imencode('.jpg', combined_frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/set_params', methods=['POST'])
def set_params():
    """
    Update HSV and blob size parameters from the UI sliders.
    """
    config.hsv_min = (
        int(request.form.get("h_min", config.hsv_min[0])),
        int(request.form.get("s_min", config.hsv_min[1])),
        int(request.form.get("v_min", config.hsv_min[2]))
    )
    config.hsv_max = (
        int(request.form.get("h_max", config.hsv_max[0])),
        int(request.form.get("s_max", config.hsv_max[1])),
        int(request.form.get("v_max", config.hsv_max[2]))
    )
    config.min_area = int(request.form.get("min_area", config.min_area))
    return ('', 204)  # Empty response to indicate success

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
    
    # Stop PWM signals and cleanup GPIO
    pwm_motor_1.stop()
    pwm_motor_2.stop()
    GPIO.cleanup()

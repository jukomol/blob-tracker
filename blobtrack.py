from flask import Flask, render_template, Response, request, jsonify
import cv2
import numpy as np
import json
import RPi.GPIO as GPIO
import config
from blob_detection import capture_blob_error
from pid_controller import PIDController
from imu_integration import get_swing_correction

# GPIO pin configuration for ESC control
MOTOR_PIN_1 = 12
MOTOR_PIN_2 = 13

# Initialize Flask app
app = Flask(__name__)

# Initialize default camera index and open camera
camera_index = 0  # Default camera index
camera = cv2.VideoCapture(camera_index)

# Function to scan connected cameras
def scan_cameras(max_index=10):
    available_cameras = []
    for index in range(max_index):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                available_cameras.append(index)
            cap.release()
    return available_cameras

# Initialize PID controller for x and y axes
pid = PIDController(config.Kp, config.Ki, config.Kd, config.dt)

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN_1, GPIO.OUT)
GPIO.setup(MOTOR_PIN_2, GPIO.OUT)

# Initialize PWM for motors at 50Hz frequency
pwm_motor_1 = GPIO.PWM(MOTOR_PIN_1, 50)
pwm_motor_2 = GPIO.PWM(MOTOR_PIN_2, 50)
pwm_motor_1.start(0)
pwm_motor_2.start(0)

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
    apply PID control, and send the frame with overlay and binary mask in real-time.
    """
    global camera
    while True:
        # Capture a frame
        ret, frame = camera.read()
        if not ret:
            break

        # Convert to HSV and apply thresholds to create a binary mask
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, (config.h_min, config.s_min, config.v_min),
                                       (config.h_max, config.s_max, config.v_max))

        # Perform blob detection and calculate error
        error_x, error_y, frame = capture_blob_error(camera)
        
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

        # Convert mask to a 3-channel image for display purposes
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Combine the original frame with the binary mask side by side
        combined_frame = np.hstack((frame, mask_colored))

        # Encode the combined frame to JPEG
        ret, buffer = cv2.imencode('.jpg', combined_frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

# Flask route for main page with available cameras passed to HTML template
@app.route('/')
def index():
    available_cameras = scan_cameras()
    return render_template('index.html', available_cameras=available_cameras)

# Route to change the active camera based on user selection
@app.route('/set_camera', methods=['POST'])
def set_camera():
    """
    Set the selected camera based on user input.
    """
    global camera, camera_index
    camera_index = int(request.form.get("camera_index", 0))  # Get camera index from form data
    camera.release()  # Release the current camera
    camera = cv2.VideoCapture(camera_index)  # Set and initialize the new camera
    return jsonify(success=True)

@app.route('/video_feed')
def video_feed():
    """
    Video streaming route.
    """
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/set_params', methods=['POST'])
def set_params():
    """
    Update HSV and blob size parameters from the UI sliders.
    """
    # Update HSV thresholds
    config.h_min = int(request.form.get("h_min", config.h_min))
    config.s_min = int(request.form.get("s_min", config.s_min))
    config.v_min = int(request.form.get("v_min", config.v_min))
    config.h_max = int(request.form.get("h_max", config.h_max))
    config.s_max = int(request.form.get("s_max", config.s_max))
    config.v_max = int(request.form.get("v_max", config.v_max))
    
    # Update blob size constraints
    config.min_area = int(request.form.get("min_area", config.min_area))
    config.max_area = int(request.form.get("max_area", config.max_area))
    
    # Update shape detection parameters
    config.min_circularity = float(request.form.get("min_circularity", config.min_circularity))
    config.min_convexity = float(request.form.get("min_convexity", config.min_convexity))
    config.min_inertia = float(request.form.get("min_inertia", config.min_inertia))

    return ('', 204)  # Empty response to indicate success

@app.route('/motor_calibration', methods=['POST'])
def motor_calibration():
    """
    Calibrate the motor by setting up PWM signals.
    """
    calibrate_motor(MOTOR_PIN_1, MOTOR_PIN_2, GPIO)
    return "Motor calibration completed successfully!"

@app.route('/imu_calibration', methods=['POST'])
def imu_calibration():
    """
    Calibrate the IMU for swing correction.
    """
    calibrate_imu()
    return "IMU calibration completed successfully!"

if __name__ == '__main__':
    try:
        app.run(debug=True, host='0.0.0.0', port=5000)
    finally:
        pwm_motor_1.stop()
        pwm_motor_2.stop()
        GPIO.cleanup()

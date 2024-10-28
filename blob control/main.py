from flask import Flask, render_template, Response, request
import cv2
import numpy as np
import json
import RPi.GPIO as GPIO
import config
from blob_detection import capture_blob_error, get_mask
from pid_controller import PIDController
from imu_integration import get_swing_correction
from motor_calibration import calibrate_motor
from imu_calibration import calibrate_imu

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

# Ensure previous PWM objects are stopped and initialize PWM
try:
    pwm_motor_1 = GPIO.PWM(MOTOR_PIN_1, 50)  # 50Hz frequency for ESC
    pwm_motor_2 = GPIO.PWM(MOTOR_PIN_2, 50)  # 50Hz frequency for ESC
    pwm_motor_1.start(0)
    pwm_motor_2.start(0)
except RuntimeError as e:
    print("Error initializing PWM:", e)
    GPIO.cleanup()
    exit(1)

# Load motor calibration data
with open("motor_calibration.json", "r") as f:
    calibration_data = json.load(f)
motor_1_cal = calibration_data["motor_1"]
motor_2_cal = calibration_data["motor_2"]

def pwm_from_microseconds(microseconds):
    return (microseconds / 20000) * 100

def set_motor_speed(motor_pwm, speed_pwm, motor_cal):
    pwm_value = max(motor_cal["min_pwm"], min(motor_cal["max_pwm"], speed_pwm))
    motor_pwm.ChangeDutyCycle(pwm_from_microseconds(pwm_value))

def generate_frames():
    while True:
        error_x, error_y, frame = capture_blob_error(camera)
        mask = get_mask(frame)
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        if error_x is not None and error_y is not None:
            output_x, output_y = pid.compute(error_x, error_y)
            swing_correction_x, swing_correction_y, _ = get_swing_correction(swing_gain=0.1)
            output_x += swing_correction_x
            output_y += swing_correction_y

            motor_speed_1 = 1500 + output_x - output_y
            motor_speed_2 = 1500 - output_x + output_y

            set_motor_speed(pwm_motor_1, motor_speed_1, motor_1_cal)
            set_motor_speed(pwm_motor_2, motor_speed_2, motor_2_cal)

        combined_frame = np.hstack((frame, mask_colored))
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
    config.max_area = int(request.form.get("max_area", config.max_area))
    config.min_circularity = float(request.form.get("min_circularity", config.min_circularity)) / 100
    config.min_convexity = float(request.form.get("min_convexity", config.min_convexity)) / 100
    config.min_inertia = float(request.form.get("min_inertia", config.min_inertia)) / 100
    return ('', 204)  # Empty response to indicate success

@app.route('/motor_calibration', methods=['POST'])
def motor_calibration():
    calibrate_motor(MOTOR_PIN_1, MOTOR_PIN_2, GPIO)
    return "Motor calibration completed successfully!"

@app.route('/imu_calibration', methods=['POST'])
def imu_calibration():
    calibrate_imu()
    return "IMU calibration completed successfully!"

if __name__ == '__main__':
    try:
        app.run(debug=True, host='0.0.0.0', port=5000)
    finally:
        pwm_motor_1.stop()
        pwm_motor_2.stop()
        GPIO.cleanup()

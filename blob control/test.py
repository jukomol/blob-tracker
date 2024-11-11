from flask import Flask, render_template, Response, request, jsonify
import cv2
import numpy as np
import json
import RPi.GPIO as GPIO
import config
from blob_detection import capture_blob_error
from pid_controller import PIDController
from imu_integration import get_swing_correction
from logger import log_data  # Import the log_data function
import math
import time
import lsm6ds33  # Assuming this is the IMU library
import os

# Initialize IMU
imu = lsm6ds33.LSM6DS33()  # Replace with the correct initialization if necessary

# Load calibration data from file
calibration_file = 'imu_calibration.json'
if os.path.exists(calibration_file):
    with open(calibration_file, 'r') as f:
        calibration_data = json.load(f)
        accel_bias = calibration_data["accelerometer_bias"]
        gyro_bias = calibration_data["gyroscope_bias"]
else:
    print("Calibration file not found. Please calibrate the IMU first.")
    accel_bias = {"x": 0.0, "y": 0.0, "z": 0.0}
    gyro_bias = {"x": 0.0, "y": 0.0, "z": 0.0}

# Function to get corrected acceleration data
def get_corrected_acceleration():
    raw_accel = imu.get_acceleration()  # Returns (ax, ay, az)
    corrected_accel = (
        raw_accel[0] - accel_bias['x'],
        raw_accel[1] - accel_bias['y'],
        raw_accel[2] - accel_bias['z']
    )
    return corrected_accel

# Function to get corrected gyroscope data
def get_corrected_gyroscope():
    raw_gyro = imu.get_gyroscope()  # Returns (gx, gy, gz)
    corrected_gyro = (
        raw_gyro[0] - gyro_bias['x'],
        raw_gyro[1] - gyro_bias['y'],
        raw_gyro[2] - gyro_bias['z']
    )
    return corrected_gyro

# Calculate swing correction based on gyroscope data (for swinging motion)
def get_swing_correction(swing_gain=0.1):
    # Retrieve corrected gyroscope data
    corrected_gyro = get_corrected_gyroscope()

    swing_correction_x = math.sin(corrected_gyro[0]*math.pi/180)
    swing_correction_y = math.cos(corrected_gyro[1]*math.pi/180)
    swing_correction_z = math.sin(corrected_gyro[2]*math.pi/180)

    # Apply gain to create a swing correction factor for each axis
    swing_correction_x = corrected_gyro[0] * swing_gain
    swing_correction_y = corrected_gyro[1] * swing_gain
    swing_correction_z = corrected_gyro[2] * swing_gain

    return swing_correction_x, swing_correction_y, swing_correction_z

def gyro_to_roll_pitch_yaw(dt=0.01):
    corrected_gyro = get_corrected_gyroscope()
    gyro_x = corrected_gyro[0]
    gyro_y = corrected_gyro[1]
    gyro_z = corrected_gyro[2]
    # Integrate gyro values to get roll, pitch, and yaw
    roll = np.cumsum(gyro_x) * dt  # Integrate x-axis gyro
    pitch = np.cumsum(gyro_y) * dt  # Integrate y-axis gyro
    yaw = np.cumsum(gyro_z) * dt    # Integrate z-axis gyro

    return roll, pitch, yaw

def gyro_to_cartesian(r=1):
    roll, pitch, yaw = gyro_to_roll_pitch_yaw(dt=0.01)
    # Convert to Cartesian coordinates
    x = r * np.cos(pitch) * np.cos(yaw)
    y = r * np.cos(pitch) * np.sin(yaw)
    z = r * np.sin(pitch)

    return x, y, z

if __name__ == "__main__":
    # Test the IMU integration functions
    print("Corrected Acceleration:", get_corrected_acceleration())
    print("Corrected Gyroscope:", get_corrected_gyroscope())
    print("Swing Correction:", get_swing_correction())
    while True:
        # swing_correction_x, swing_correction_y, swing_correction_z = get_swing_correction(swing_gain=1)
        # print("Swing Correction X:", swing_correction_x)
        # print("Swing Correction Y:", swing_correction_y)
        # print("Swing Correction Z:", swing_correction_z)
        # time.sleep(1)
        roll, pitch, yaw = gyro_to_roll_pitch_yaw(dt=0.01)
        #print("Roll:", roll)
        #print("Pitch:", pitch)
        print("Yaw:", yaw)
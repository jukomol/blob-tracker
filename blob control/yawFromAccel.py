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
rangePerDigitAccel = 0b01000000  # 104 Hz, 2g
rangePerDigitGyro = 0b01000000   # 104 Hz, 250 dps

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

def window_filter(data, window_size=3):
    return np.convolve(data, np.ones(window_size) / window_size, mode='valid')

# Function to calculate yaw angle
def calculate_yaw(acc_data, dx=0.5):
    # Apply window filter to raw acceleration data
    filtered_data = window_filter(acc_data)
    
    # Compute position changes (x) using Simpson's 1/3 rule
    x_positions = np.zeros(len(filtered_data))
    for i in range(1, len(filtered_data)):
        x_positions[i] = simpsons_one_third(filtered_data[:i+1], dx)
    
    # Calculate rotation angles (ω)
    omega = 2 * np.arcsin(x_positions / (2 * R))
    
    # Compute total yaw angle by summing ω
    yaw_angle = np.sum(omega)
    return np.degrees(yaw_angle)  # Convert to degrees

# Simpson's 1/3 integration method
def simpsons_one_third(data, dx):
    n = len(data) - 1
    if n % 2 == 1:  # n must be even
        raise ValueError("Simpson's rule requires an even number of intervals.")
    h = dx
    integral = (data[0] + data[-1] + 4 * sum(data[1:-1:2]) + 2 * sum(data[2:-2:2])) * h / 3
    return integral

# Function to get corrected gyroscope data
def get_corrected_gyroscope():
    raw_gyro = imu.get_gyroscope()  # Returns (gx, gy, gz)
    corrected_gyro = (
        raw_gyro[0] - gyro_bias['x'],
        raw_gyro[1] - gyro_bias['y'],
        raw_gyro[2] - gyro_bias['z']
    )
    return corrected_gyro

def main():
    acc_data = []
    dx = 0.5  # Distance step in cm
    window_size = 3  # Filter window size
    
    try:
        while True:
            corrected_accel = get_corrected_acceleration()
            ax = corrected_accel[0]  # Only x-axis is needed for yaw
            
            # Collect acceleration data
            acc_data.append(ax)
            if len(acc_data) >= window_size:
                # Calculate yaw after enough data is collected
                yaw = calculate_yaw(acc_data, dx)
                print(f"Calculated Yaw Angle: {yaw:.2f} degrees")
            
            # Adjust sleep for sampling rate (adjust for 104 Hz)
            time.sleep(1 / 104)
    except KeyboardInterrupt:
        print("Yaw calculation terminated.")

def normalizedAccel():
    corrected_accel = get_corrected_acceleration()
    aX = corrected_accel[0]*rangePerDigitAccel* 9.81
    aY = corrected_accel[1]*rangePerDigitAccel* 9.81
    aZ = corrected_accel[2]*rangePerDigitAccel* 9.81
    return aX, aY, aZ

def normalizedGyro():
    corrected_gyro = get_corrected_gyroscope()
    gX = corrected_gyro[0]*rangePerDigitGyro
    gY = corrected_gyro[1]*rangePerDigitGyro
    gZ = corrected_gyro[2]*rangePerDigitGyro
    return gX, gY, gZ

# Helper function to normalize an angle to the range [0, 360)
def normalize_angle_360(angle):
    angle = angle % 360  # Wrap angle around 360 degrees
    return angle if angle >= 0 else angle + 360

# Calculate swing correction based on gyroscope data (for swinging motion)
def get_swing_correction(swing_gain=0.1):
    # Retrieve corrected gyroscope data
    corrected_gyro = get_corrected_gyroscope()

    swing_correction_x = math.sin(corrected_gyro[0] * (math.pi / 180))
    swing_correction_y = math.cos(corrected_gyro[1] * (math.pi / 180))
    swing_correction_z = math.sin(corrected_gyro[2] * (math.pi / 180))

    # Apply gain to create a swing correction factor for each axis
    swing_correction_x *= swing_gain
    swing_correction_y *= swing_gain
    swing_correction_z *= swing_gain

    return swing_correction_x, swing_correction_y, swing_correction_z

# Convert gyroscope readings to roll, pitch, and yaw (yaw normalized to [0, 360))
def gyro_to_roll_pitch_yaw(dt=0.1):
    corrected_gyro = get_corrected_gyroscope()

    gyro_x = corrected_gyro[0]
    gyro_y = corrected_gyro[1]
    gyro_z = corrected_gyro[2]

    # Integrate gyro values to calculate roll, pitch, and yaw
    roll = np.cumsum([gyro_x]) * dt  # Integrate x-axis gyro
    pitch = np.cumsum([gyro_y]) * dt  # Integrate y-axis gyro
    yaw = np.cumsum([gyro_z]) * dt    # Integrate z-axis gyro

    # Normalize yaw to the range [0, 360)
    yaw = normalize_angle_360(yaw[-1])  # Use the last value of the cumulative sum

    return roll[-1], pitch[-1], yaw

# Convert gyro-based roll, pitch, yaw to Cartesian coordinates
def gyro_to_cartesian(r=1):
    roll, pitch, yaw = gyro_to_roll_pitch_yaw(dt=0.01)
    # Convert to Cartesian coordinates
    x = r * np.cos(np.radians(pitch)) * np.cos(np.radians(yaw))
    y = r * np.cos(np.radians(pitch)) * np.sin(np.radians(yaw))
    z = r * np.sin(np.radians(pitch))

    return x, y, z

if __name__ == "__main__":
    main()
    # Test the IMU integration functions
    #print("Corrected Acceleration:", get_corrected_acceleration())
    #print("Corrected Gyroscope:", get_corrected_gyroscope())
    #print("Swing Correction:", get_swing_correction())
    #while True:
        #roll, pitch, yaw = gyro_to_roll_pitch_yaw(dt=0.1)
        #x, y, z = gyro_to_cartesian(r=1)

        #print("Roll:", roll)
        #print("Pitch:", pitch)
        #print("Yaw (0-360):", yaw)  # Yaw in the range 0-360
        #print("X:", x, "Y:", y, "Z:", z)
        #time.sleep(1)

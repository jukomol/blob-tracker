import json
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

    # Apply gain to create a swing correction factor for each axis
    swing_correction_x = corrected_gyro[0] * swing_gain
    swing_correction_y = corrected_gyro[1] * swing_gain
    swing_correction_z = corrected_gyro[2] * swing_gain

    return swing_correction_x, swing_correction_y, swing_correction_z

if __name__ == "__main__":
    # Test the IMU integration functions
    print("Corrected Acceleration:", get_corrected_acceleration())
    print("Corrected Gyroscope:", get_corrected_gyroscope())
    print("Swing Correction:", get_swing_correction())

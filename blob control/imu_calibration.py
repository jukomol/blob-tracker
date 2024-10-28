import json
import time
import lsm6ds33  # Assuming this is the provided IMU library

# Initialize the IMU sensor
imu = lsm6ds33.LSM6DS33()  # Replace with correct initialization if necessary

# Number of samples for calibration
SAMPLE_COUNT = 100
DELAY = 0.05  # Delay in seconds between samples

def calibrate_imu():
    # Initialize accumulators for accelerometer and gyroscope biases
    accel_bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    gyro_bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    print("Starting IMU calibration... Please keep the sensor still.")

    for i in range(SAMPLE_COUNT):
        # Read the current acceleration and gyroscope data
        accel_data = imu.get_acceleration()  # Returns a tuple (ax, ay, az)
        gyro_data = imu.get_gyroscope()      # Returns a tuple (gx, gy, gz)

        # Accumulate the data
        accel_bias['x'] += accel_data[0]
        accel_bias['y'] += accel_data[1]
        accel_bias['z'] += accel_data[2]
        
        gyro_bias['x'] += gyro_data[0]
        gyro_bias['y'] += gyro_data[1]
        gyro_bias['z'] += gyro_data[2]

        # Wait a short time before taking the next sample
        time.sleep(DELAY)

    # Calculate average values for bias
    accel_bias = {axis: value / SAMPLE_COUNT for axis, value in accel_bias.items()}
    gyro_bias = {axis: value / SAMPLE_COUNT for axis, value in gyro_bias.items()}

    print("Calibration complete.")
    print(f"Accelerometer Bias: {accel_bias}")
    print(f"Gyroscope Bias: {gyro_bias}")

    # Save calibration data to a JSON file
    calibration_data = {
        "accelerometer_bias": accel_bias,
        "gyroscope_bias": gyro_bias
    }
    with open('imu_calibration.json', 'w') as f:
        json.dump(calibration_data, f, indent=4)
    print("Calibration data saved to imu_calibration.json")

if __name__ == "__main__":
    calibrate_imu()

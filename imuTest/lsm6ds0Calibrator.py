import time
import json
import os
from lsm6ds0 import LSM6DS0  # Import the LSM6DS0 library

class LSM6DS0Calibrator:
    CALIBRATION_FILE = "lsm6ds0_calibration.json"

    def __init__(self, sensor):
        """
        Initialize the calibrator with the LSM6DS0 sensor instance.
        :param sensor: An instance of the LSM6DS0 class.
        """
        self.sensor = sensor
        self.accel_offsets = [0, 0, 0]  # X, Y, Z offsets for accelerometer
        self.gyro_offsets = [0, 0, 0]   # X, Y, Z offsets for gyroscope
        self.load_calibration_data()

    def calibrate_accel(self, samples=100, delay=0.05):
        """
        Calibrate the accelerometer by averaging raw readings while the sensor is stationary.
        :param samples: Number of samples to take for calibration.
        :param delay: Delay between each sample in seconds.
        """
        print("Calibrating accelerometer. Please keep the sensor stationary.")

        accel_sum = [0, 0, 0]

        for _ in range(samples):
            accel_raw = self.sensor.readRawAccel()

            for i in range(3):
                accel_sum[i] += accel_raw[i]

            time.sleep(delay)

        self.accel_offsets = [x / samples for x in accel_sum]

        print(f"Accelerometer offsets: {self.accel_offsets}")

    def calibrate_gyro(self, samples=100):
        """
        Calibrate the gyroscope using the method from the LSM6DS0 library.
        :param samples: Number of samples to take for calibration.
        """
        self.sensor.calibrateGyro(samples)  # Delegate to the sensor's calibration method
        self.gyro_offsets = self.sensor.gyro_offsets
        print(f"Gyroscope offsets: {self.gyro_offsets}")

    def save_calibration_data(self):
        """
        Save calibration offsets to a JSON file.
        """
        calibration_data = {
            "accel_offsets": self.accel_offsets,
            "gyro_offsets": self.gyro_offsets
        }
        with open(self.CALIBRATION_FILE, "w") as file:
            json.dump(calibration_data, file, indent=4)
        print(f"Calibration data saved to {self.CALIBRATION_FILE}.")

    def load_calibration_data(self):
        """
        Load calibration offsets from a JSON file if it exists.
        """
        if os.path.exists(self.CALIBRATION_FILE):
            with open(self.CALIBRATION_FILE, "r") as file:
                calibration_data = json.load(file)
                self.accel_offsets = calibration_data.get("accel_offsets", [0, 0, 0])
                self.gyro_offsets = calibration_data.get("gyro_offsets", [0, 0, 0])
                self.sensor.gyro_offsets = self.gyro_offsets
            print(f"Calibration data loaded from {self.CALIBRATION_FILE}.")
        else:
            print("No calibration data found. Using default offsets.")

    def get_calibrated_data(self):
        """
        Get calibrated data from the sensor.
        :return: Tuple of calibrated accelerometer and gyroscope data.
        """
        accel_raw = self.sensor.readRawAccel()
        gyro_raw = self.sensor.readRawGyro()

        # Apply offsets
        accel_calibrated = [raw - offset for raw, offset in zip(accel_raw, self.accel_offsets)]
        gyro_calibrated = [(raw - offset) for raw, offset in zip(gyro_raw, self.gyro_offsets)]

        return accel_calibrated, gyro_calibrated
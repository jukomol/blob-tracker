from lsm6ds0 import LSM6DS0
from lsm6ds0Calibrator import LSM6DS0Calibrator
import time

def main():
    # Initialize the LSM6DS0 sensor
    sensor = LSM6DS0()

    # Create a calibrator instance
    calibrator = LSM6DS0Calibrator(sensor)

    # Ask the user if recalibration is needed
    recalibrate = input("Do you want to recalibrate the sensor? (y/n): ").strip().lower()
    if recalibrate == "y":
        print("Recalibrating accelerometer...")
        calibrator.calibrate_accel(samples=200, delay=0.1)
        print("Recalibrating gyroscope...")
        calibrator.calibrate_gyro(samples=200)
        calibrator.save_calibration_data()

    # Continuously read calibrated data
    print("\nReading calibrated data from the sensor...\n")
    try:
        while True:
            accel_data, gyro_data = calibrator.get_calibrated_data()
            print(f"Calibrated Accelerometer: {accel_data}")
            print(f"Calibrated Gyroscope: {gyro_data}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting program.")

if __name__ == "__main__":
    main()

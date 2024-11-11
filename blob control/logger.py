# logger.py

import csv
import os
from datetime import datetime

class Logger:
    _instance = None  # Singleton instance
    _log_file_initialized = False  # Track if log file was initialized

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(Logger, cls).__new__(cls)
            cls._instance.initialize_log_file()
        return cls._instance

    def initialize_log_file(self):
        # Generate a unique log file name based on the current timestamp
        log_dir = "logs"
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = os.path.join(log_dir, f"log_data_{timestamp}.csv")

        # Column headers for the log file
        fieldnames = [
            "timestamp", 
            "blob_x", "blob_y",
            "target_x", "target_y",
            "Kp", "Ki", "Kd", "dt",
            "output_x", "output_y",
            "motor_speed_1", "motor_speed_2"
        ]

        # Initialize the log file and write headers
        if not Logger._log_file_initialized:
            with open(self.log_file_path, mode="w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=fieldnames)
                writer.writeheader()
            Logger._log_file_initialized = True  # Set to True to avoid reinitializing

    def log_data(self, blob_x, blob_y, target_x, target_y, Kp, Ki, Kd, dt, output_x, output_y, motor_speed_1, motor_speed_2):
        """
        Log the provided data to a CSV file with a unique timestamped filename.
        """
        # Prepare the data row with a timestamp
        data_row = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "blob_x": blob_x,
            "blob_y": blob_y,
            "target_x": target_x,
            "target_y": target_y,
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "dt": dt,
            "output_x": output_x,
            "output_y": output_y,
            "motor_speed_1": motor_speed_1,
            "motor_speed_2": motor_speed_2
        }

        # Append the data to the CSV file
        with open(self.log_file_path, mode="a", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=data_row.keys())
            writer.writerow(data_row)

# Initialize the logger instance
logger = Logger()

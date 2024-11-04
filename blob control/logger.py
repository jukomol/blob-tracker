# logger.py

import csv
import os
from datetime import datetime

# Generate a unique log file name based on the current timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_file_path = f"log_data_{timestamp}.csv"

# Column headers for the log file
fieldnames = [
    "timestamp", 
    "blob_x", "blob_y",
    "target_x", "target_y",
    "Kp", "Ki", "Kd", "dt",
    "output_x", "output_y",
    "motor_speed_1", "motor_speed_2", "speed_pwm"
]

# Initialize the log file and write headers
with open(log_file_path, mode="w", newline="") as file:
    writer = csv.DictWriter(file, fieldnames=fieldnames)
    writer.writeheader()

def log_data(blob_x, blob_y, target_x, target_y, Kp, Ki, Kd, dt, output_x, output_y, motor_speed_1, motor_speed_2, speed_pwm):
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
        "motor_speed_2": motor_speed_2,
        "speed_pwm": speed_pwm
    }

    # Append the data to the CSV file
    with open(log_file_path, mode="a", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writerow(data_row)

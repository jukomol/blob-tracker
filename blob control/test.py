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


swing_correction_x, swing_correction_y, swing_correction_z = get_swing_correction(swing_gain=0)

print("Swing Correction X:", swing_correction_x)
print("Swing Correction Y:", swing_correction_y)
print("Swing Correction Z:", swing_correction_z)
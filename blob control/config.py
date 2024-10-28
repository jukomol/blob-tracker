# config.py

# Initial HSV thresholds for blob detection
hsv_min = (35, 100, 100)  # Initial values for hue, saturation, and value (HSV) minimums
hsv_max = (85, 255, 255)  # Initial values for hue, saturation, and value (HSV) maximums

# Blob area thresholds
min_area = 400   # Minimum blob area for detection
max_area = 20000 # Maximum blob area for detection

# Additional blob detection parameters (default values)
min_circularity = 0.3    # Circularity threshold (range [0, 1])
min_convexity = 0.5      # Convexity threshold (range [0, 1])
min_inertia_ratio = 0.3  # Inertia ratio threshold (range [0, 1])

# Target center position for error calculation
target_x = 320  # Assuming 640x480 resolution, center is at (320, 240)
target_y = 240

# PID Controller parameters (can be tuned)
Kp = 0.1  # Proportional gain
Ki = 0.01 # Integral gain
Kd = 0.05 # Derivative gain

# Initial PID control variables
initial_prev_error_x = 0
initial_prev_error_y = 0
initial_integral_x = 0
initial_integral_y = 0
dt = 0.1  # Control loop time step (adjust based on actual timing)

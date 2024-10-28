# config.py

# config.py

# HSV color space thresholds for blob detection
h_min = 35   # Hue minimum
s_min = 100  # Saturation minimum
v_min = 100  # Value minimum
h_max = 85   # Hue maximum
s_max = 255  # Saturation maximum
v_max = 255  # Value maximum

# Blob size constraints
min_area = 400     # Minimum area for blob detection
max_area = 20000   # Maximum area for blob detection

# Blob shape detection parameters
min_circularity = 0.30  # Minimum circularity (0 to 1)
min_convexity = 0.50    # Minimum convexity (0 to 1)
min_inertia = 0.30      # Minimum inertia ratio (0 to 1)

# PID control parameters for blob centering
Kp = 0.1   # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.05  # Derivative gain
dt = 0.1   # Time interval for PID controller

# Initial PID control variables
initial_prev_error_x = 0
initial_prev_error_y = 0
initial_integral_x = 0
initial_integral_y = 0
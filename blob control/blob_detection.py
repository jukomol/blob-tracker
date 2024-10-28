# blob_detection.py

import cv2
import numpy as np
import config

# Default HSV and area parameters
hsv_params = {
    "h_min": config.hsv_min[0],
    "s_min": config.hsv_min[1],
    "v_min": config.hsv_min[2],
    "h_max": config.hsv_max[0],
    "s_max": config.hsv_max[1],
    "v_max": config.hsv_max[2],
    "min_area": 400  # Minimum area threshold for detecting a blob
}

def get_mask(frame):
    """
    Apply HSV masking to detect specific colors.
    """
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv_frame,
        (hsv_params["h_min"], hsv_params["s_min"], hsv_params["v_min"]),
        (hsv_params["h_max"], hsv_params["s_max"], hsv_params["v_max"])
    )
    return mask

def detect_blobs(frame):
    """
    Detect blobs in the given frame and return blob count and first blob coordinates.
    """
    mask = get_mask(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    blob_count = 0
    first_blob_position = None

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > hsv_params["min_area"]:
            blob_count += 1
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if blob_count == 1:
                first_blob_position = (int(x), int(y))
            # Draw the blob circle and center point
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 0), -1)

    return frame, blob_count, first_blob_position, mask

def capture_blob_error(cap):
    """
    Capture a frame, detect the blob, and calculate the error from the target center.
    Returns error_x, error_y, and the processed frame with drawn blobs.
    """
    ret, frame = cap.read()
    if not ret:
        return None, None, None  # Return if frame capture failed
    
    # Detect blobs
    processed_frame, blob_count, first_blob_position, mask = detect_blobs(frame)
    
    if blob_count > 0 and first_blob_position:
        blob_x, blob_y = first_blob_position
        error_x = config.target_x - blob_x
        error_y = config.target_y - blob_y
        return error_x, error_y, processed_frame
    
    # No blob detected, return None for errors
    return None, None, frame

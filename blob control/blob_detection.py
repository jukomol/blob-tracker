# blob_detection.py

import cv2
import numpy as np
import config
import math

def get_mask(frame):
    """
    Apply HSV masking to detect specific colors.
    """
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv_frame,
        (config.h_min, config.s_min, config.v_min),
        (config.h_max, config.s_max, config.v_max)
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
        
        # Check area constraints
        if config.min_area < area < config.max_area:
            # Calculate shape features
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * (area / (perimeter ** 2)) if perimeter > 0 else 0
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            convexity = area / hull_area if hull_area > 0 else 0
            moments = cv2.moments(contour)
            inertia = (moments["mu02"] / moments["mu20"]) if moments["mu20"] > 0 else 0

            # Check shape constraints
            if (circularity > config.min_circularity and
                convexity > config.min_convexity and
                inertia > config.min_inertia):

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

def blob_position(cap):
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

        return blob_x, blob_y, processed_frame
    
    # No blob detected, return None for errors
    return None, None, frame

def angle_calculate(cap):
    """
    Calculate the angle between the x-axis and the line connecting the origin and the point (x, y).
    """
    ret, frame = cap.read()
    if not ret:
        return None, None  # Return if frame capture failed
    
    # Detect blobs
    processed_frame, blob_count, first_blob_position, mask = detect_blobs(frame)
    
    if blob_count > 0 and first_blob_position:
        blob_x, blob_y = first_blob_position
        target_x = config.target_x
        target_y = config.target_y
        AB = math.sqrt((target_x - blob_x)**2 + (target_y - blob_y)**2)
        BC = math.sqrt((blob_x - blob_y)**2 + (target_y - blob_y)**2)
        x = math.arcsin(BC/AB)

        return x, processed_frame
    
    # No blob detected, return None for errors
    return None, frame




import cv2
import numpy as np

# Default values for HSV and blob parameters
hsv_params = {
    "h_min": 35, "s_min": 100, "v_min": 100,
    "h_max": 85, "s_max": 255, "v_max": 255,
    "min_area": 400  # Minimum area threshold for detecting a blob
}

def get_mask(frame):
    """
    Apply HSV masking to detect specific colors.
    """
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, (hsv_params["h_min"], hsv_params["s_min"], hsv_params["v_min"]),
                                  (hsv_params["h_max"], hsv_params["s_max"], hsv_params["v_max"]))
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

import cv2
import numpy as np

# Font to write text overlay
font = cv2.FONT_HERSHEY_SIMPLEX

# Read test image
frame = cv2.imread("ball2.jpeg")

# Create window for trackbars
cv2.namedWindow("Trackbars")

# Function to update values
def nothing(x):
    pass

# Create trackbars for HSV thresholds
cv2.createTrackbar("H Min", "Trackbars", 20, 179, nothing)
cv2.createTrackbar("S Min", "Trackbars", 120, 255, nothing)
cv2.createTrackbar("V Min", "Trackbars", 120, 255, nothing)
cv2.createTrackbar("H Max", "Trackbars", 49, 179, nothing)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, nothing)

# Create trackbars for blob size
cv2.createTrackbar("Min Area", "Trackbars", 400, 5000, nothing)
cv2.createTrackbar("Max Area", "Trackbars", 20000, 50000, nothing)

while True:
    # Get current positions of trackbars
    h_min = cv2.getTrackbarPos("H Min", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")
    
    min_area = cv2.getTrackbarPos("Min Area", "Trackbars")
    max_area = cv2.getTrackbarPos("Max Area", "Trackbars")

    # Set HSV thresholds based on trackbar values
    hsvMin = (h_min, s_min, v_min)
    hsvMax = (h_max, s_max, v_max)

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Apply HSV thresholds
    mask = cv2.inRange(hsv, hsvMin, hsvMax)

    # Erode and dilate to reduce noise
    mask = cv2.erode(mask, None, iterations=3)
    mask = cv2.dilate(mask, None, iterations=3)

    # Adjust detection parameters
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 100

    # Filter by Area
    params.filterByArea = True
    params.minArea = min_area
    params.maxArea = max_area

    # Detect blobs
    detector = cv2.SimpleBlobDetector_create(params)

    # Invert the mask
    reversemask = 255 - mask

    # Run blob detection
    keypoints = detector.detect(reversemask)

    # Get the number of blobs found
    blobCount = len(keypoints)

    # Create a copy of the frame to display
    display_frame = frame.copy()

    # Write the number of blobs found
    text = "Count=" + str(blobCount)
    cv2.putText(display_frame, text, (5, 25), font, 1, (0, 255, 0), 2)

    if blobCount > 0:
        # Write X position of first blob
        blob_x = keypoints[0].pt[0]
        text2 = "X=" + "{:.2f}".format(blob_x)
        cv2.putText(display_frame, text2, (5, 50), font, 1, (0, 255, 0), 2)

        # Write Y position of first blob
        blob_y = keypoints[0].pt[1]
        text3 = "Y=" + "{:.2f}".format(blob_y)
        cv2.putText(display_frame, text3, (5, 75), font, 1, (0, 255, 0), 2)

        # Write Size of first blob
        blob_size = keypoints[0].size
        text4 = "S=" + "{:.2f}".format(blob_size)
        cv2.putText(display_frame, text4, (5, 100), font, 1, (0, 255, 0), 2)

        # Draw circle to indicate the blob
        cv2.circle(display_frame, (int(blob_x), int(blob_y)), int(blob_size / 2), (0, 255, 0), 2)

    # Show the images
    cv2.imshow("Blob detection", display_frame)
    cv2.imshow("Mask", mask)

    # Wait for key press to exit
    if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
        break

cv2.destroyAllWindows()

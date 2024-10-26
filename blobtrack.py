import cv2
import numpy as np

# Define radius range in pixels
min_radius_pixels = 25
max_radius_pixels = 200

# Initialize video capture for real-time detection
cap = cv2.VideoCapture(1)  # Use the webcam

# Define the green color range in HSV (you can adjust these values as needed)
lower_green = np.array([35, 100, 100])
upper_green = np.array([85, 255, 255])

while True:
    # Read frame from the webcam
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for green color
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize blob count for the specified size range
    blob_count = 0

    # Loop through each detected blob
    for i, contour in enumerate(contours):
        # Calculate minimum enclosing circle to get radius
        ((cx, cy), radius) = cv2.minEnclosingCircle(contour)

        # Filter blobs by radius size
        if min_radius_pixels <= radius <= max_radius_pixels:
            blob_count += 1  # Increment blob count for valid blobs

            # Draw the contour and center
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 255), 2)  # Outline the blob
            cv2.circle(frame, (int(cx), int(cy)), 5, (255, 0, 0), -1)  # Mark center

            # Add position info overlay
            position_text = f"Blob {blob_count}: x={int(cx)}, y={int(cy)}, radius={int(radius)}"
            cv2.putText(frame, position_text, (10, 60 + blob_count*30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Show blob count for blobs within the specified size range
    overlay_text = f"Blob Count (5-200 px): {blob_count}"
    cv2.putText(frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Show the result
    cv2.imshow("Green Blob Detection", frame)

    # Exit loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()

import cv2

# Initialize the video capture
camera = cv2.VideoCapture(1)  # Change to 0 if you want to use the default camera

# Get the width and height of the frame
frame_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Camera Resolution: {frame_width}x{frame_height}")

# Release the camera when done
camera.release()

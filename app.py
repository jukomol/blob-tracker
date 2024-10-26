from flask import Flask, render_template, Response, request
import cv2
import numpy as np

app = Flask(__name__)

# Initialize the video capture with the USB camera
camera = cv2.VideoCapture(1)  # Change to 0 for the default camera

# Default values for HSV parameters specifically for green detection
hsv_params = {
    "h_min": 35, "s_min": 100, "v_min": 100,
    "h_max": 85, "s_max": 255, "v_max": 255,
    "min_area": 400  # Minimum area threshold for detecting a blob
}

# Generate frames with contour-based blob detection in the binary mask
def generate_frames():
    while True:
        # Capture frame
        success, frame = camera.read()
        if not success:
            break

        # Get HSV and blob size parameters
        h_min, s_min, v_min = hsv_params["h_min"], hsv_params["s_min"], hsv_params["v_min"]
        h_max, s_max, v_max = hsv_params["h_max"], hsv_params["s_max"], hsv_params["v_max"]
        min_area = hsv_params["min_area"]

        # Convert to HSV and apply mask for green color
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, (h_min, s_min, v_min), (h_max, s_max, v_max))
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter and draw detected blobs
        blob_count = 0
        first_blob_position = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:  # Only consider contours with area above the threshold
                blob_count += 1
                # Get the center and radius of the contour for drawing
                (x, y), radius = cv2.minEnclosingCircle(contour)
                
                # Save the position of the first blob for display
                if blob_count == 1:
                    first_blob_position = (int(x), int(y))
                
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)  # Draw yellow circle
                # Draw center point of the blob
                cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 0), -1)  # Green dot at center

        # Display blob count and the X, Y position of the first blob if detected
        overlay_text = f"Blob Count: {blob_count}"
        if first_blob_position:
            overlay_text += f", X: {first_blob_position[0]}, Y: {first_blob_position[1]}"
        cv2.putText(frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Combine the frame with blobs and the mask side by side
        combined_frame = np.hstack((frame, mask_colored))

        # Encode to JPEG
        ret, buffer = cv2.imencode('.jpg', combined_frame)
        if not ret:
            continue

        # Yield the frame in the correct format to be streamed
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/set_params', methods=['POST'])
def set_params():
    # Update HSV and blob size parameters from the slider values
    hsv_params["h_min"] = int(request.form.get("h_min", 35))
    hsv_params["s_min"] = int(request.form.get("s_min", 100))
    hsv_params["v_min"] = int(request.form.get("v_min", 100))
    hsv_params["h_max"] = int(request.form.get("h_max", 85))
    hsv_params["s_max"] = int(request.form.get("s_max", 255))
    hsv_params["v_max"] = int(request.form.get("v_max", 255))
    hsv_params["min_area"] = int(request.form.get("min_area", 400))
    return ('', 204)  # Empty response

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)

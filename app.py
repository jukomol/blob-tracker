from flask import Flask, render_template, Response, request
import cv2
import numpy as np

app = Flask(__name__)

# Initialize the video capture with the USB camera
camera = cv2.VideoCapture(1)  # Change to 0 for the default camera

# Default values for HSV and blob parameters
hsv_params = {
    "h_min": 35, "s_min": 100, "v_min": 100,
    "h_max": 85, "s_max": 255, "v_max": 255,
    "min_area": 400, "max_area": 20000
}

# Set up the SimpleBlobDetector parameters
def setup_blob_detector(min_area, max_area):
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 0
    params.maxThreshold = 100
    params.filterByArea = True
    params.minArea = min_area
    params.maxArea = max_area
    detector = cv2.SimpleBlobDetector_create(params)
    return detector

# Generate frames with real-time tuning
def generate_frames():
    while True:
        # Capture frame
        success, frame = camera.read()
        if not success:
            break

        # Get HSV and blob size parameters
        h_min = hsv_params["h_min"]
        s_min = hsv_params["s_min"]
        v_min = hsv_params["v_min"]
        h_max = hsv_params["h_max"]
        s_max = hsv_params["s_max"]
        v_max = hsv_params["v_max"]
        min_area = hsv_params["min_area"]
        max_area = hsv_params["max_area"]

        # Convert to HSV and apply mask
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, (h_min, s_min, v_min), (h_max, s_max, v_max))
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Setup blob detector with current min and max area
        detector = setup_blob_detector(min_area, max_area)
        keypoints = detector.detect(mask)

        # Draw blobs and overlay blob info
        frame_with_blobs = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255),
                                             cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        blob_count = len(keypoints)

        # Display blob count and first blob coordinates
        if blob_count > 0:
            blob_x, blob_y = int(keypoints[0].pt[0]), int(keypoints[0].pt[1])
            overlay_text = f"Blob Count: {blob_count}, X: {blob_x}, Y: {blob_y}"
        else:
            overlay_text = "Blob Count: 0"

        # Overlay text at the top right corner
        cv2.putText(frame_with_blobs, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Combine the frame with blobs and the mask side by side
        combined_frame = np.hstack((frame_with_blobs, mask_colored))

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
    hsv_params["max_area"] = int(request.form.get("max_area", 20000))
    return ('', 204)  # Empty response

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)

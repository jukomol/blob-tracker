from flask import Flask, render_template, Response, request
import cv2
import numpy as np
from blobDetector import detect_blobs, hsv_params  # Import blob detection functions

app = Flask(__name__)

# Initialize the video capture with the USB camera
camera = cv2.VideoCapture(1)  # Change to 0 for the default camera

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break

        # Get frame dimensions and calculate the center
        frame_height, frame_width = frame.shape[:2]
        center_x, center_y = frame_width // 2, frame_height // 2

        # Draw a cross at the center of the frame
        cross_size = 10
        cv2.line(frame, (center_x - cross_size, center_y), (center_x + cross_size, center_y), (255, 0, 0), 2)
        cv2.line(frame, (center_x, center_y - cross_size), (center_x, center_y + cross_size), (255, 0, 0), 2)

        # Perform blob detection
        frame, blob_count, first_blob_position, mask = detect_blobs(frame)
        
        # Display blob count and coordinates of the first blob
        overlay_text = f"Blob Count: {blob_count}"
        if first_blob_position:
            overlay_text += f", X: {first_blob_position[0]}, Y: {first_blob_position[1]}"
        cv2.putText(frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Combine the frame with blobs and the mask side by side
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined_frame = np.hstack((frame, mask_colored))

        # Encode to JPEG
        ret, buffer = cv2.imencode('.jpg', combined_frame)
        if not ret:
            continue

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

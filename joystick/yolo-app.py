from flask import Flask, Response, render_template
import cv2
from ultralytics import YOLO

app = Flask(__name__)

# MJPEG stream URL with authentication
url = "http://admin:admin123@192.168.1.112:80/cgi-bin/mjpg/video.cgi?channel=1&subtype=1"

def load_model(model_path):
    model = YOLO(model_path)
    return model

def detect_objects(model, frame):
    results = model(frame)
    return results

def draw_bounding_boxes(frame, results, model):
    for result in results:
        boxes = result.boxes.cpu().numpy()
        for box in boxes:
            r = box.xyxy[0].astype(int)
            cls = int(box.cls[0])
            conf = box.conf[0]
            cv2.rectangle(frame, (r[0], r[1]), (r[2], r[3]), (0, 255, 0), 2)
            cv2.putText(frame, f'{model.names[cls]} {conf:.2f}', (r[0], r[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    return frame

def generate_frames():
    cap = cv2.VideoCapture(url)
    
    if not cap.isOpened():
        print("Error: Could not open video stream")
        return
    model_path = 'best.pt'  # You can use any pre-trained model like yolov8s.pt, yolov8m.pt, etc.
    model = load_model(model_path)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame")
            break
        # Perform object detection
        results = detect_objects(model, frame)

        # Draw bounding boxes and labels on the frame
        frame = draw_bounding_boxes(frame, results, model)
        
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
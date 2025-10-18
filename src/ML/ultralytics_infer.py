import cv2
import time
import torch
from ultralytics import YOLO

# Enable GPU if available
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# Load YOLO model once and move to device
model = YOLO("best.pt")  # or "yolov8n.pt", "yolov8s.pt", etc.
model.to(device)
model.fuse()  # fuse Conv+BN layers for faster inference

# Video setup
video_path = "video.mp4"
cap = cv2.VideoCapture(video_path)

# Get video properties
fps = cap.get(cv2.CAP_PROP_FPS)
width, height = int(cap.get(3)), int(cap.get(4))
print(f"Original Video FPS: {fps:.2f}, Resolution: {width}x{height}")

# change resolution for faster processing if needed
width, height = 640, 360
cap.set(3, width)
cap.set(4, height)

print(f"Video FPS: {fps:.2f}, Resolution: {width}x{height}")

# ROI (Region of Interest)
# roi_x1, roi_y1, roi_x2, roi_y2 = 200, 170, 700, 500
roi_x1, roi_y1, roi_x2, roi_y2 = 0, 0, width, height  # Full frame

# Optional: resize ROI for faster inference
resize_scale = 0.7  # 70% of ROI size, tweak for speed vs accuracy

prev_time = time.time()
frame_skip = 2  # process every frame (increase for faster playback)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Optionally skip frames for more speed
    if frame_skip > 1:
        for _ in range(frame_skip - 1):
            cap.grab()

    # Crop ROI
    roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]

    # Resize ROI for faster processing
    if resize_scale != 1.0:
        roi = cv2.resize(roi, (0, 0), fx=resize_scale, fy=resize_scale)

    # Run inference
    results = model.predict(
        roi,
        verbose=False,
        conf=0.5,
        iou=0.5,
        imgsz=640,  # smaller imgsz = faster
        device=device
    )

    # Draw detections
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].int().tolist()
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = f"{model.names[cls]} {conf:.2f}"

            # Adjust coordinates if resized
            if resize_scale != 1.0:
                x1 = int(x1 / resize_scale)
                x2 = int(x2 / resize_scale)
                y1 = int(y1 / resize_scale)
                y2 = int(y2 / resize_scale)

            # Map back to full frame
            x1 += roi_x1
            x2 += roi_x1
            y1 += roi_y1
            y2 += roi_y1

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    # Draw ROI box
    cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 0, 0), 2)

    # FPS calculation
    current_time = time.time()
    fps_text = 1 / (current_time - prev_time)
    prev_time = current_time
    cv2.putText(frame, f"FPS: {fps_text:.2f}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Show frame
    cv2.imshow("YOLO Detection with ROI", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
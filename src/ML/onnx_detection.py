import cv2
import numpy as np
import onnxruntime as ort
import time

# ====== CONFIG ======
MODEL_PATH = "best.onnx"
LABELS = ["class1", "class2", ""]  # Replace with your classes
IMG_SIZE = 320
CONF_THRESHOLD = 0.5

# ROI (x1, y1, x2, y2)
ROI = (200, 100, 900, 600)  # Change as needed
# ====================

# Load model
session = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])
input_name = session.get_inputs()[0].name
output_name = session.get_outputs()[0].name

def preprocess(frame):
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))  # CHW
    return np.expand_dims(img, 0)  # BCHW

def xywh2xyxy(x):
    y = np.zeros_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2
    y[:, 1] = x[:, 1] - x[:, 3] / 2
    y[:, 2] = x[:, 0] + x[:, 2] / 2
    y[:, 3] = x[:, 1] + x[:, 3] / 2
    return y

def nms(boxes, scores, iou_threshold=0.45):
    """Basic Non-Max Suppression (NMS)"""
    indices = []
    boxes = boxes.astype(np.float32)
    scores = scores.astype(np.float32)
    order = scores.argsort()[::-1]
    while order.size > 0:
        i = order[0]
        indices.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = ((boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1]) +
                 (boxes[order[1:], 2] - boxes[order[1:], 0]) *
                 (boxes[order[1:], 3] - boxes[order[1:], 1]) - inter)
        iou = inter / (union + 1e-6)
        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]
    return indices

cap = cv2.VideoCapture("video.mp4")
fps_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    # Draw ROI border
    x1_roi, y1_roi, x2_roi, y2_roi = ROI
    cv2.rectangle(frame, (x1_roi, y1_roi), (x2_roi, y2_roi), (255, 255, 0), 2)
    roi_frame = frame[y1_roi:y2_roi, x1_roi:x2_roi]

    # Preprocess ROI
    input_img = preprocess(roi_frame)

    # Run inference
    output = session.run([output_name], {input_name: input_img})[0]  # (1, 7, 8400)
    preds = np.squeeze(output).T  # (8400, 7)

    boxes = preds[:, :4]
    scores = preds[:, 4]
    class_probs = preds[:, 5:]
    class_ids = np.argmax(class_probs, axis=1)
    confidences = scores * class_probs[np.arange(len(class_probs)), class_ids]

    mask = confidences > CONF_THRESHOLD
    boxes = boxes[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]

    # Convert coordinates
    boxes = xywh2xyxy(boxes)
    boxes[:, [0, 2]] *= (x2_roi - x1_roi) / IMG_SIZE
    boxes[:, [1, 3]] *= (y2_roi - y1_roi) / IMG_SIZE
    boxes[:, [0, 2]] += x1_roi
    boxes[:, [1, 3]] += y1_roi

    # Apply NMS
    if len(boxes) > 0:
        idxs = nms(boxes, confidences)
        boxes = boxes[idxs]
        confidences = confidences[idxs]
        class_ids = class_ids[idxs]

    # Draw detections
    for (x1, y1, x2, y2), conf, cls in zip(boxes, confidences, class_ids):
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        label = f"{LABELS[int(cls)]}: {conf:.2f}"
        cv2.putText(frame, label, (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Calculate FPS
    now = time.time()
    fps = 1 / (now - fps_time)
    fps_time = now
    cv2.putText(frame, f"FPS: {fps:.1f}", (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    cv2.imshow("ONNX Detection + ROI", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
import cv2
import numpy as np
import os

# --- CONFIG ---
VIDEO_PATH = "video2.mp4"  # or 0 for webcam
OBS_REGION = [85, 155, 555, 320]  # [x1, y1, x2, y2]
TARGET_SIZE = 320
SAVE_DIR = "output_images"

SKIP_FRAMES = 20

# --- SETUP ---
os.makedirs(SAVE_DIR, exist_ok=True)
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print("❌ Cannot open video")
    exit()

frame_count = 0
saved_count = 0

print("🎥 Press 's' to save, 'k' to skip 20 frames, 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("✅ End of video or camera feed.")
        break

    frame = cv2.resize(frame, (640, 480))
    roi = frame[OBS_REGION[1]:OBS_REGION[3], OBS_REGION[0]:OBS_REGION[2]]  # Crop ROI

    # --- Convert to grayscale ---
    roi = cv2.bilateralFilter(roi, d=9, sigmaColor=75, sigmaSpace=75)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # --- Edge detection ---
    edges = cv2.Canny(blurred, 50, 150)    

    # --- Invert black/white ---
    inverted = cv2.bitwise_not(edges)

    # --- Convert to BGR for display and saving ---
    edge_bgr = cv2.cvtColor(inverted, cv2.COLOR_GRAY2BGR)

    # --- Resize or pad to exactly 320x320 ---
    h, w, _ = edge_bgr.shape
    scale = min(TARGET_SIZE / w, TARGET_SIZE / h)
    resized = cv2.resize(edge_bgr, (int(w * scale), int(h * scale)))

    # Create white canvas
    canvas = np.ones((TARGET_SIZE, TARGET_SIZE, 3), dtype=np.uint8) * 255
    h_r, w_r, _ = resized.shape
    y_offset = (TARGET_SIZE - h_r) // 2
    x_offset = (TARGET_SIZE - w_r) // 2
    canvas[y_offset:y_offset + h_r, x_offset:x_offset + w_r] = resized

    # --- Display ---
    cv2.imshow("Edges (Inverted, 320x320)", canvas)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        filename = os.path.join(SAVE_DIR, f"frame_{saved_count:04d}.jpg")
        cv2.imwrite(filename, canvas)
        print(f"💾 Saved: {filename}")
        saved_count += 1

    elif key == ord('k'):
        for _ in range(SKIP_FRAMES):
            cap.read()
        print(f"⏭️ Skipped {SKIP_FRAMES} frames.")

    elif key == ord('q'):
        print("👋 Exiting.")
        break

    frame_count += 1

cap.release()
cv2.destroyAllWindows()
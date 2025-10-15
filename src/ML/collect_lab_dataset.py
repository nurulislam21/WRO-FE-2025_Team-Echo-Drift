import cv2
import numpy as np
import csv
import time

# --- Settings ---
USE_CAMERA = "webcam"  # "webcam" or "picam"
OUTPUT_CSV = "lab_dataset.csv"
CAM_INDEX = 0  # for webcam
WINDOW_NAME = "LAB Collector"

# --- Create CSV and header if not exists ---
with open(OUTPUT_CSV, mode="w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "L", "a", "b", "label"])

# --- Mouse callback for clicks ---
clicked_points = []


def click_event(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((x, y))


# --- Camera initialization ---
cap = None
picam2 = None

if USE_CAMERA.lower() == "webcam":
    print("[INFO] Using USB/Webcam")
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("❌ Cannot open webcam")
        exit()

elif USE_CAMERA.lower() == "picam":
    from picamera2 import Picamera2
    print("[INFO] Using PiCamera2")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (1280, 720)})
    #config = picam2.create_preview_configuration(main={"size": (1920, 1080)})
    #config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)  # small delay to warm up camera

else:
    print("❌ Invalid camera type. Choose 'webcam' or 'picam'.")
    exit()

cv2.namedWindow(WINDOW_NAME)
cv2.setMouseCallback(WINDOW_NAME, click_event)

print("\n[INFO] Click on objects to record LAB values.")
print("[INFO] Press 'q' to quit, 'c' to clear clicks, 's' to save LAB values.\n")

while True:
    if USE_CAMERA.lower() == "webcam":
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to capture frame from webcam")
            break
    else:
        frame = picam2.capture_array()
        # PiCamera2 gives RGB by default; convert to BGR for OpenCV consistency
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    frame_copy = frame.copy()

    # Draw markers
    for (x, y) in clicked_points:
        cv2.circle(frame_copy, (x, y), 4, (0, 0, 255), -1)

    cv2.imshow(WINDOW_NAME, frame_copy)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):  # quit
        break

    elif key == ord("c"):  # clear points
        clicked_points = []
        print("[INFO] Cleared selected points.")

    elif key == ord("s"):  # save LAB values
        if not clicked_points:
            print("[WARN] No points selected.")
            continue

        # Convert to LAB        
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        label = input("Enter color label for these points: ").strip()
        rows = []

        for (x, y) in clicked_points:
            if y >= lab.shape[0] or x >= lab.shape[1]:
                print(f"[WARN] Point ({x}, {y}) out of bounds, skipped.")
                continue
            L, a, b = lab[y, x]
            rows.append([time.time(), float(L), float(a), float(b), label])
            print(f"Saved LAB=({L},{a},{b}) label={label}")

        # Append to CSV
        with open(OUTPUT_CSV, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(rows)

        print(f"[INFO] {len(rows)} samples saved.\n")
        clicked_points = []

# --- Cleanup ---
if USE_CAMERA.lower() == "webcam" and cap:
    cap.release()
elif picam2:
    picam2.stop()

cv2.destroyAllWindows()
print(f"[INFO] Data collection finished. Saved to {OUTPUT_CSV}")

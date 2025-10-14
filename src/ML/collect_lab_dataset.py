import cv2
import numpy as np
import csv
import time

# --- Settings ---
OUTPUT_CSV = "lab_dataset.csv"
CAM_INDEX = 0  # change if multiple cameras
WINDOW_NAME = "LAB Collector"

# --- Create CSV and header if not exists ---
with open(OUTPUT_CSV, mode="a", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "L", "a", "b", "label"])

# --- Mouse callback for clicks ---
clicked_points = []

def click_event(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((x, y))

# --- Start camera ---
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

cv2.namedWindow(WINDOW_NAME)
cv2.setMouseCallback(WINDOW_NAME, click_event)

print("\n[INFO] Click on objects to record LAB values.")
print("[INFO] Press 'q' to quit, 'c' to clear clicks.\n")

while True:
    ret, frame = cap.read()
    frame_copy = frame.copy()

    if not ret:
        print("❌ Failed to capture frame")
        break

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
            # region = lab[y-2:y+3, x-2:x+3].reshape(-1, 3)
            # L, a, b = np.mean(region, axis=0)
            L, a, b = lab[y, x]
            rows.append([time.time(), float(L), float(a), float(b), label])
            print(f"Saved LAB=({L},{a},{b}) label={label}")

        # Append to CSV
        with open(OUTPUT_CSV, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(rows)

        print(f"[INFO] {len(rows)} samples saved.\n")
        clicked_points = []

cap.release()
cv2.destroyAllWindows()
print(f"[INFO] Data collection finished. Saved to {OUTPUT_CSV}")
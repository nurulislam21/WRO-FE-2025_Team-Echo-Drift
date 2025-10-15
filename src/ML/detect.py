import cv2
import numpy as np
import time

# --- Choose camera ---
USE_CAMERA = "webcam"   # "picam" or "webcam"
CAM_INDEX = 0           # for USB webcam

# --- Define fitted curve equation ---
def fitted_b(a):
    a = np.asarray(a, dtype=np.float32)
    b = (-0.0070 * a ** 2) + (3.0256 * a) - 164.0170
    b[(a < 160) | (a > 224)] = np.inf
    return b

# --- Initialize camera ---
cap = None
picam2 = None

if USE_CAMERA.lower() == "picam":
    from picamera2 import Picamera2
    print("[INFO] Using PiCamera2")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (1280, 720)})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

elif USE_CAMERA.lower() == "webcam":
    print("[INFO] Using Webcam")
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("❌ Cannot open webcam")
        exit()

else:
    print("❌ Invalid USE_CAMERA value. Choose 'picam' or 'webcam'.")
    exit()

# --- Parameters ---
threshold = 5  # distance tolerance in 'b' axis, tune this for your lighting conditions

print("[INFO] Press 'q' to quit.\n")

# --- Main loop ---
while True:
    if USE_CAMERA.lower() == "picam":
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to capture frame.")
            break

    # Convert frame to Lab color space
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    L, a, b = cv2.split(lab)

    # Convert to float for precision
    a = a.astype(np.float32)
    b = b.astype(np.float32)

    # Predict b from the fitted curve
    b_pred = fitted_b(a)

    # Compute absolute difference between actual b and predicted b
    diff = np.abs(b - b_pred)

    # Threshold the difference to get mask
    mask = (diff < threshold).astype(np.uint8) * 255

    # Optional: Apply morphological operations to clean mask
    mask = cv2.medianBlur(mask, 5)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours
    result = frame.copy()
    cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

    # Display results
    cv2.imshow("Mask", mask)
    cv2.imshow("Detected Color Contours", result)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# --- Cleanup ---
if picam2:
    picam2.stop()
if cap:
    cap.release()

cv2.destroyAllWindows()
print("[INFO] Stopped successfully.")

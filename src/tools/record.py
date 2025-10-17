import cv2
import time

# --- Settings ---
USE_CAMERA = "picam"  # "webcam" or "picam"
CAM_INDEX = 0          # USB webcam index
OUTPUT_FILE = f"recorded_{int(time.time())}.mp4"  # output filename
FPS = 30               # target FPS
RESOLUTION = (800, 600)  # width x height

# --- Initialize camera ---
cap = None
picam2 = None

if USE_CAMERA.lower() == "webcam":
    print("[INFO] Using USB Webcam")
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    cap.set(cv2.CAP_PROP_FPS, FPS)

    if not cap.isOpened():
        print("? Cannot open webcam")
        exit()

elif USE_CAMERA.lower() == "picam":
    from picamera2 import Picamera2

    print("[INFO] Using PiCamera2")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": RESOLUTION, "format": "RGB888"})
    picam2.configure(config)
    picam2.set_controls(
        {
            "ExposureTime": 6000,
            "AnalogueGain": 9.4,
            "AeEnable": False,
            "AwbEnable": False,
            "FrameDurationLimits": (40000, 40000),
            "ColourGains": (0.8, 1.2),
            "Contrast": 1.1,
            "Saturation": 3.5,
        }
    )
    picam2.start()
    time.sleep(1)
else:
    print("? Invalid USE_CAMERA value. Choose 'webcam' or 'picam'.")
    exit()

# --- Video writer ---
fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # or "XVID"
out = cv2.VideoWriter(OUTPUT_FILE, fourcc, FPS, RESOLUTION)

print("[INFO] Recording... Press 'q' to stop.")

while True:
    if USE_CAMERA.lower() == "webcam":
        ret, frame = cap.read()
        if not ret:
            print("? Failed to read frame from webcam")
            break
    else:
        frame = picam2.capture_array()
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BG)

    # Write frame to output
    out.write(frame)

    # Display live preview
    cv2.imshow("Recording", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# --- Cleanup ---
if cap:
    cap.release()
if picam2:
    picam2.stop()
out.release()
cv2.destroyAllWindows()

print(f"[INFO] Video saved to {OUTPUT_FILE}")
import cv2
import time

# --- Settings ---
USE_CAMERA = "picam"  # "webcam" or "picam"
CAM_INDEX = "/dev/video0"  # For webcam
FPS = 30
RESOLUTION = (800, 600)

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

    # Load camera settings if available
    # try:
    #     with open("camera_settings.json", "r") as f:
    #         import json
    #         settings = json.load(f)
    #         picam2.set_controls(settings)
    #         print("Loaded camera settings from file.")
    # except FileNotFoundError:
    #     print("No camera settings file found. Using default settings.")

    picam2.start()
    time.sleep(1)

else:
    print("? Invalid USE_CAMERA value.")
    exit()

# --- Recording control ---
is_recording = False
out = None
OUTPUT_FILE = None

print("[INFO] Press 's' to START recording, 'q' to STOP and exit.")

while True:
    # --- Read frame ---
    if USE_CAMERA.lower() == "webcam":
        ret, frame = cap.read()
        if not ret:
            print("? Webcam frame read error.")
            break
    else:
        frame = picam2.capture_array()
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # --- Write to file only if recording ---
    if is_recording and out is not None:
        out.write(frame)

    # --- Display preview ---
    cv2.imshow("Live Preview", frame)

    key = cv2.waitKey(1) & 0xFF

    # --- Start Recording ---
    if key == ord("s") and not is_recording:
        OUTPUT_FILE = f"recorded_{int(time.time())}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(OUTPUT_FILE, fourcc, FPS, RESOLUTION)
        is_recording = True
        print(f"[INFO] Recording STARTED → {OUTPUT_FILE}")

    # --- Quit & Save ---
    if key == ord("q"):
        print("[INFO] Stopping and saving video...")
        break

# --- Cleanup ---
if cap:
    cap.release()
if picam2:
    picam2.stop()

if out:
    out.release()

cv2.destroyAllWindows()

if OUTPUT_FILE:
    print(f"[INFO] Video saved: {OUTPUT_FILE}")
else:
    print("[INFO] No recording was saved.")
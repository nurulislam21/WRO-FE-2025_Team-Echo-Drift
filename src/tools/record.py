import cv2
import time
from picamera2 import Picamera2

# --- Initialize camera ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (800, 600)})
picam2.configure(config)
picam2.set_controls(
            {
                "ExposureTime": 36000,
                "AnalogueGain": 24.0,
                "AeEnable": False,
                "AwbEnable": False,
                "FrameDurationLimits": (40000, 40000),
            }
        )
picam2.start()
time.sleep(1)
print("[INFO] Camera started at 800x600")
print("[INFO] Press 's' to start/stop recording, 'q' to quit.")

# --- FPS tracking ---
prev_time = time.time()
fps = 0.0

# --- Recording setup ---
recording = False
video_writer = None
output_file = None
frame_size = (800, 600)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # .mp4 output

while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # --- FPS calculation ---
    curr_time = time.time()
    fps = 1.0 / (curr_time - prev_time)
    prev_time = curr_time

    # --- Draw FPS ---
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # --- If recording, write frame to file ---
    if recording and video_writer:
        video_writer.write(frame)
        cv2.putText(frame, "REC", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # --- Show frame ---
    cv2.imshow("Picamera2 Preview", frame)

    key = cv2.waitKey(1) & 0xFF

    # --- Start/stop recording ---
    if key == ord('s'):
        if not recording:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_file = f"record_{timestamp}.mp4"
            video_writer = cv2.VideoWriter(output_file, fourcc, 30.0, frame_size)
            recording = True
            print(f"[INFO] Recording started → {output_file}")
        else:
            recording = False
            if video_writer:
                video_writer.release()
                print(f"[INFO] Recording stopped & saved → {output_file}")
                video_writer = None

    elif key == ord('q'):
        print("[INFO] Quitting...")
        break

# --- Cleanup ---
if video_writer:
    video_writer.release()
picam2.stop()
cv2.destroyAllWindows()
print("[INFO] Done.")
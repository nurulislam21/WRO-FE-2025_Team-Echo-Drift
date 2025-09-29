import sys
import serial
import cv2
import time
import tkinter as tk
from threading import Thread

# --- Arduino setup ---
arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, dsrdtr=True)
time.sleep(2)
arduino.write(b"0,-1,95\n")

# --- Camera setup ---
camera = sys.argv[1] == "--camera" if len(sys.argv) > 1 else False
if camera:
    from picamera2 import Picamera2

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
    picam2.configure(config)
    picam2.set_controls(
        {
            "ExposureTime": 11000,
            "AnalogueGain": 16.0,
            "AeEnable": False,
            "AwbEnable": False,
            "FrameDurationLimits": (40000, 40000),
        }
    )
    picam2.start()

# --- Control variables ---
speed = 0
angle = 95  # default straight

# --- Tkinter GUI ---
root = tk.Tk()
root.title("Car Controller")

lbl_status = tk.Label(root, text="Speed: 0 | Angle: 95", font=("Arial", 14))
lbl_status.pack(pady=10)

# Speed buttons
def forward():
    global speed
    speed = 80

def backward():
    global speed
    speed = -80

def stop():
    global speed
    speed = 0 

frame_speed = tk.Frame(root)
frame_speed.pack(pady=5)

btn_forward = tk.Button(frame_speed, text="Forward", command=forward, width=10, height=2)
btn_forward.grid(row=0, column=1, padx=5)

btn_stop = tk.Button(frame_speed, text="■ Stop", command=stop, width=10, height=2, bg="red", fg="white")
btn_stop.grid(row=1, column=1, padx=5)

btn_backward = tk.Button(frame_speed, text="Backward", command=backward, width=10, height=2)
btn_backward.grid(row=2, column=1, padx=5)

# Steering slider
def update_angle(val):
    global angle
    angle = int(val)    

steering_slider = tk.Scale(
    root,
    from_=30,
    to=160,
    orient="horizontal",
    length=300,
    label="Steering Angle",
    command=update_angle
)
steering_slider.set(95)  # default straight
steering_slider.pack(pady=10)

# --- Send command to Arduino ---
def send_command():
    while True:
        global speed, angle
        cmd = f"{speed},-1,{angle}\n"
        arduino.write(cmd.encode())
        print(f"Sent command: {cmd.strip()}")
        lbl_status.config(text=f"Speed: {speed} | Angle: {angle}")
        time.sleep(0.2)

# --- Camera loop (runs in a thread) ---
def camera_loop():
    while camera:
        frame = picam2.capture_array()
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    if camera:
        cv2.destroyAllWindows()

if camera:
    Thread(target=camera_loop, daemon=True).start()

# Start sending commands in a separate thread
Thread(target=send_command, daemon=True).start()
# Run GUI
try:
    root.mainloop()
finally:
    arduino.close()
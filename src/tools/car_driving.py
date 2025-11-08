
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
    config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    # load camera settings from file
    try:
        with open("camera_settings.json", "r") as f:
            import json
            settings = json.load(f)
            picam2.set_controls(settings)
            print("Loaded camera settings from file.")
    except FileNotFoundError:
        # Default settings if no file found
        print("No camera settings file found. Using default settings.")
        sys.exit(1)
  
    picam2.start()

# --- Control variables ---
speed = 0
angle = 95  # default straight
keyboard_active = False  # track if keyboard is controlling

# --- Debounce variables (only for Up/Down) ---
DEBOUNCE_TIME = 300  # milliseconds to wait before applying key release
release_timers = {}  # track pending release timers for Up/Down keys

# --- Tkinter GUI ---
root = tk.Tk()
root.title("Car Controller")

# --- Set window position ---
# Change these values to position the window where you want
window_x = 100  # X coordinate (distance from left edge of screen)
window_y = 100  # Y coordinate (distance from top edge of screen)
root.geometry(f"+{window_x}+{window_y}")

lbl_status = tk.Label(root, text="Speed: 0 | Angle: 95", font=("Arial", 14))
lbl_status.pack(pady=10)

lbl_controls = tk.Label(root, text="Use Arrow Keys or Mouse", font=("Arial", 10), fg="blue")
lbl_controls.pack()

# --- Joystick Canvas ---
canvas_size = 200
center = canvas_size // 2
radius = 30  # joystick knob radius
max_offset = 70  # max distance from center

canvas = tk.Canvas(root, width=canvas_size, height=canvas_size, bg="lightgray")
canvas.pack(pady=10)

# draw outer boundary + knob
canvas.create_oval(center - max_offset, center - max_offset,
                   center + max_offset, center + max_offset,
                   outline="black", width=2)
knob = canvas.create_oval(center - radius, center - radius,
                          center + radius, center + radius,
                          fill="blue")

# --- Joystick behavior ---
def move_knob(event):
    global speed, angle, keyboard_active
    keyboard_active = False  # mouse takes priority
    
    dx = event.x - center
    dy = event.y - center

    # limit knob inside max_offset circle
    dist = (dx**2 + dy**2) ** 0.5
    if dist > max_offset:
        dx = dx * max_offset / dist
        dy = dy * max_offset / dist

    # move knob
    canvas.coords(knob,
                  center + dx - radius, center + dy - radius,
                  center + dx + radius, center + dy + radius)

    # map dx → angle (30–160)
    angle = int(95 + (dx / max_offset) * 65)  # center=95
    angle = max(30, min(160, angle))

    # map dy → speed (-80 to 80) (invert Y for natural feel)
    speed = int(-(dy / max_offset) * 80)
    speed = max(-80, min(80, speed))

def reset_knob(event=None):
    global speed, angle, keyboard_active
    if not keyboard_active:  # only reset if keyboard isn't active
        # reset to center
        canvas.coords(knob,
                      center - radius, center - radius,
                      center + radius, center + radius)
        speed = 0
        angle = 95

canvas.bind("<B1-Motion>", move_knob)  # drag with mouse
canvas.bind("<ButtonRelease-1>", reset_knob)  # release → reset to center

# --- Keyboard controls ---
keys_pressed = set()

def on_key_press(event):
    global speed, angle, keyboard_active
    keyboard_active = True
    
    # Cancel any pending release timer for this key (only for Up/Down)
    if event.keysym in ["Up", "Down"] and event.keysym in release_timers:
        root.after_cancel(release_timers[event.keysym])
        del release_timers[event.keysym]
    
    keys_pressed.add(event.keysym)
    update_from_keyboard()

def on_key_release(event):
    global speed, angle, keyboard_active
    
    # Debounce logic ONLY for Up/Down keys
    if event.keysym in ["Up", "Down"]:
        # Schedule a delayed release
        def apply_release():
            if event.keysym in keys_pressed:
                keys_pressed.remove(event.keysym)
            if event.keysym in release_timers:
                del release_timers[event.keysym]
            update_from_keyboard()
            
            # if no keys pressed, reset to neutral
            if not keys_pressed:
                keyboard_active = False
                speed = 0
                angle = 95
                reset_knob()
        
        # Cancel existing timer if any
        if event.keysym in release_timers:
            root.after_cancel(release_timers[event.keysym])
        
        # Schedule the release after debounce time
        timer_id = root.after(DEBOUNCE_TIME, apply_release)
        release_timers[event.keysym] = timer_id
    
    else:
        # Immediate release for Left/Right keys (no debounce)
        if event.keysym in keys_pressed:
            keys_pressed.remove(event.keysym)
        update_from_keyboard()
        
        # if no keys pressed, reset to neutral
        if not keys_pressed:
            keyboard_active = False
            speed = 0
            angle = 95
            reset_knob()

def update_from_keyboard():
    global speed, angle
    
    # Speed control (forward/backward) - independent of turning
    if "Up" in keys_pressed:
        speed = 45  # forward
    elif "Down" in keys_pressed:
        speed = -45  # backward
    else:
        speed = 0
    
    # Angle control (left/right) - independent of speed
    if "Left" in keys_pressed:
        angle = 35  # turn left
    elif "Right" in keys_pressed:
        angle = 155  # turn right
    else:
        angle = 95  # straight
    
    # Update visual knob position based on keyboard input
    if keyboard_active:
        # map speed to dy
        dy = -(speed / 80.0) * max_offset
        # map angle to dx
        dx = ((angle - 95) / 65.0) * max_offset
        
        canvas.coords(knob,
                      center + dx - radius, center + dy - radius,
                      center + dx + radius, center + dy + radius)

# Bind keyboard events
root.bind("<KeyPress>", on_key_press)
root.bind("<KeyRelease>", on_key_release)

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

# --- Force focus on the GUI window ---
root.lift()  # Bring window to front
root.attributes('-topmost', True)  # Make it stay on top temporarily
root.after(100, lambda: root.attributes('-topmost', False))  # Remove topmost after 100ms
root.focus_force()  # Force keyboard focus

# Run GUI
try:
    root.mainloop()
finally:
    arduino.close()
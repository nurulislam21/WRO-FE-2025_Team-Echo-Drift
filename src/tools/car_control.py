import sys
import serial
import cv2
import time
import keyboard

# Setup Arduino serial
camera = sys.argv[1] == "--camera" if len(sys.argv) > 1 else False

print("Camera:", camera)

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

arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, dsrdtr=True)
time.sleep(2)
arduino.write(b"0,-1,95\n")

speed = 0
angle = 95  # default straight

while True:
    if camera:
        frame = picam2.capture_array()

    # --- SPEED CONTROL ---
    if keyboard.is_pressed("w"):
        speed = 80
    elif keyboard.is_pressed("s"):
        speed = -80
    else:
        speed = 0

    # --- ANGLE CONTROL ---
    if keyboard.is_pressed("a"):
        angle = 30
    elif keyboard.is_pressed("d"):
        angle = 160
    else:
        angle = 95  # reset when no steering key pressed

    # print status
    print(f"Speed: {speed}, Angle: {angle}")

    # Send to Arduino
    arduino.write(f"{speed},-1,{angle}\n".encode())

    if camera:
        # Display camera
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

if camera:
    cv2.destroyAllWindows()

arduino.close()


make a GUI with tkinter instead of keyboard
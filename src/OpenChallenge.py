#
# import cv2
# import numpy as np
# import serial
# from time import sleep
# import time
# # from picamera2 import Picamera2
# # import serial
#
# # Simulated camera settings
# CAM_WIDTH = 640
# CAM_HEIGHT = 480
#
# # Region of Interest coordinates (same as original)
# ROI1 = [20, 170, 240, 220]
# ROI2 = [400, 170, 620, 220]
# ROI3 = [200, 300, 440, 350]
#
# # Color ranges
# LOWER_BLACK = np.array([0, 0, 0])
# UPPER_BLACK = np.array([180, 255, 50])
#
# LOWER_ORANGE = np.array([5, 50, 50])
# # LOWER_ORANGE = np.array([10, 100, 100])
# UPPER_ORANGE = np.array([25, 255, 255])
# LOWER_BLUE = np.array([90, 50, 50])
# # LOWER_BLUE = np.array([90, 100, 100])
# UPPER_BLUE = np.array([130, 255, 255])
#
# # Control parameters (same as original)
# kp = 0.02
# kd = 0.006
# straightConst = 92
# turnThresh = 150
# exitThresh = 1500
# tDeviation = 25
# sharpRight = straightConst - tDeviation
# sharpLeft = straightConst + tDeviation
# maxRight = straightConst - 50
# maxLeft = straightConst + 50
#
#
# def find_contours(frame, lower_color, upper_color, roi):
#     x1, y1, x2, y2 = roi
#     roi_frame = frame[y1:y2, x1:x2]
#
#     # Convert to HSV for better color segmentation
#     hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
#     mask = cv2.inRange(hsv, lower_color, upper_color)
#
#     # Morphological operations to reduce noise
#     kernel = np.ones((5, 5), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
#
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     return contours
#
#
# def max_contour(contours):
#     if len(contours) == 0:
#         return (0, None)
#     largest = max(contours, key=cv2.contourArea)
#     area = cv2.contourArea(largest)
#     return (area, largest)
#
#
# def display_roi(frame, rois, color):
#     for roi in rois:
#         x1, y1, x2, y2 = roi
#         cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
#     return frame
#
# # ser = serial.Serial('/dev/ttyACMC0', 115200, timeout=1.0)
# #     ser.flush()
#
# # Change port as needed: Windows = COM3, Linux = /dev/ttyUSB0
# # arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)  # Example for Windows
# arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.05)  # For Linux
#
# time.sleep(2)  # Wait for Arduino to reset
#
# # Send message to Arduino
# arduino.write(b'Hello Arduino\n')
#
# def main():
#     # time.sleep(3)
#     #
#     # # initialize camera
#     # picam2 = Picamera2()
#     # picam2.preview_configuration.main.size = (640, 480)
#     # picam2.preview_configuration.main.format = "RGB888"
#     # picam2.preview_configuration.controls.FrameRate = 30
#     # picam2.preview_configuration.align()
#     # picam2.configure("preview")
#     # picam2.start()
#
#     # Initialize webcam
#     # cap = picam2.capture_array()
#     cap = cv2.VideoCapture(0)
#     # cap = cv2.VideoCapture('wro2020-fe-POV2-120d-ezgif.com-resize.gif')
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
#
#     # State variables
#     lTurn = False
#     rTurn = False
#     t = 0
#     angle = straightConst
#     prevAngle = angle
#     aDiff = 0
#     prevDiff = 0
#     lDetected = False
#     debug = True
#     start = False
#     turnDir = "none"
#
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break
#
#         # Flip frame horizontally to match mirror image
#         frame = cv2.flip(frame, 1)
#
#         # Find contours
#         cListLeft = find_contours(frame, LOWER_BLACK, UPPER_BLACK, ROI1)
#         cListRight = find_contours(frame, LOWER_BLACK, UPPER_BLACK, ROI2)
#         cListOrange = find_contours(frame, LOWER_ORANGE, UPPER_ORANGE, ROI3)
#         cListBlue = find_contours(frame, LOWER_BLUE, UPPER_BLUE, ROI3)
#
#         # Get areas
#         leftArea, _ = max_contour(cListLeft)
#         rightArea, _ = max_contour(cListRight)
#         orangeArea, _ = max_contour(cListOrange)
#         blueArea, _ = max_contour(cListBlue)
#
#         # Detect colored markers
#         if orangeArea > 100:
#             lDetected = True
#             if turnDir == "none":
#                 turnDir = "right"
#                 print(turnDir)
#         elif blueArea > 100:
#             lDetected = True
#             if turnDir == "none":
#                 turnDir = "left"
#                 print(turnDir)
#
#         # Calculate steering angle
#         aDiff = rightArea - leftArea
#         angle = int(max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0))
#
#         # Turn detection logic
#         if leftArea <= turnThresh and not rTurn:
#             lTurn = True
#         elif rightArea <= turnThresh and not lTurn:
#             rTurn = True
#
#         # Turn exit logic
#         if (rightArea > exitThresh and rTurn) or (leftArea > exitThresh and lTurn):
#             lTurn = False
#             rTurn = False
#             prevDiff = 0
#             if lDetected:
#                 t += 1
#                 lDetected = False
#
#         # Steering angle clamping
#         if lTurn:
#             angle = min(max(angle, sharpLeft), maxLeft)
#         elif rTurn:
#             angle = max(min(angle, sharpRight), maxRight)
#         else:
#             angle = max(min(angle, sharpLeft), sharpRight)
#
#         # Display debug info
#         if debug:
#             debug_frame = frame.copy()
#             debug_frame = display_roi(debug_frame, [ROI1, ROI2, ROI3], (255, 0, 255))
#
#             # Draw contours
#             cv2.drawContours(debug_frame[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cListOrange, -1, (0, 255, 0), 2)
#             cv2.drawContours(debug_frame[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], cListLeft, -1, (0, 255, 0), 2)
#             cv2.drawContours(debug_frame[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], cListRight, -1, (0, 255, 0), 2)
#
#             # Display status text
#             status = f"Angle: {angle} | Turns: {t} | L: {leftArea} | R: {rightArea}"
#             cv2.putText(debug_frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
#             cv2.imshow("Debug View", debug_frame)
#
#         arduino.write(f"{angle}\n".encode())
#
#         prevDiff = aDiff
#         prevAngle = angle
#
#         # Exit conditions
#         if t >= 12 and abs(angle - straightConst) <= 20:
#             print("Lap completed!")
#             # ser.write(b"stop\n")
#             break
#
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#
#     cap.release()
#     cv2.destroyAllWindows()
#
#
# if __name__ == '__main__':
#     main()



import cv2
import numpy as np
import serial
from time import sleep
import time
import os

# CAM SETTINGS
CAM_WIDTH = 640
CAM_HEIGHT = 480

# ROI and COLOR RANGES (unchanged)
ROI1 = [20, 170, 240, 220]
ROI2 = [400, 170, 620, 220]
ROI3 = [200, 300, 440, 350]

LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 255, 50])
LOWER_ORANGE = np.array([5, 50, 50])
UPPER_ORANGE = np.array([25, 255, 255])
LOWER_BLUE = np.array([90, 50, 50])
UPPER_BLUE = np.array([130, 255, 255])

# CONTROL PARAMETERS
kp = 0.02
kd = 0.006
straightConst = 92
turnThresh = 150
exitThresh = 1500
tDeviation = 25
sharpRight = straightConst - tDeviation
sharpLeft = straightConst + tDeviation
maxRight = straightConst - 50
maxLeft = straightConst + 50


def find_contours(frame, lower_color, upper_color, roi):
    x1, y1, x2, y2 = roi
    roi_frame = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def max_contour(contours):
    if len(contours) == 0:
        return (0, None)
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    return (area, largest)

def display_roi(frame, rois, color):
    for roi in rois:
        x1, y1, x2, y2 = roi
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    return frame

# ----- SERIAL SETUP -----
def setup_serial():
    try:
        return serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.05)
    except serial.SerialException:
        try:
            return serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
        except:
            print("⚠️ Arduino not connected.")
            return None

arduino = setup_serial()
if arduino:
    time.sleep(2)
    arduino.write(b'Hello Arduino\n')


def main():
    # CAMERA SETUP
    cap = cv2.VideoCapture(0)  # Use USB camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    if not cap.isOpened():
        print("❌ Failed to open camera.")
        return

    # CONTROL STATE
    lTurn = False
    rTurn = False
    t = 0
    angle = straightConst
    prevAngle = angle
    aDiff = 0
    prevDiff = 0
    lDetected = False
    debug = True
    start = False
    turnDir = "none"

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)

        cListLeft = find_contours(frame, LOWER_BLACK, UPPER_BLACK, ROI1)
        cListRight = find_contours(frame, LOWER_BLACK, UPPER_BLACK, ROI2)
        cListOrange = find_contours(frame, LOWER_ORANGE, UPPER_ORANGE, ROI3)
        cListBlue = find_contours(frame, LOWER_BLUE, UPPER_BLUE, ROI3)

        leftArea, _ = max_contour(cListLeft)
        rightArea, _ = max_contour(cListRight)
        orangeArea, _ = max_contour(cListOrange)
        blueArea, _ = max_contour(cListBlue)

        # Marker detection
        if orangeArea > 100 and turnDir == "none":
            lDetected = True
            turnDir = "right"
            print(turnDir)
        elif blueArea > 100 and turnDir == "none":
            lDetected = True
            turnDir = "left"
            print(turnDir)

        aDiff = rightArea - leftArea
        angle = int(max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0))

        # Turn logic
        if leftArea <= turnThresh and not rTurn:
            lTurn = True
        elif rightArea <= turnThresh and not lTurn:
            rTurn = True

        if (rightArea > exitThresh and rTurn) or (leftArea > exitThresh and lTurn):
            lTurn = False
            rTurn = False
            prevDiff = 0
            if lDetected:
                t += 1
                lDetected = False

        # Clamp steering
        if lTurn:
            angle = min(max(angle, sharpLeft), maxLeft)
        elif rTurn:
            angle = max(min(angle, sharpRight), maxRight)
        else:
            angle = max(min(angle, sharpLeft), sharpRight)

        # DEBUG INFO
        if debug:
            debug_frame = frame.copy()
            debug_frame = display_roi(debug_frame, [ROI1, ROI2, ROI3], (255, 0, 255))
            cv2.drawContours(debug_frame[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cListOrange, -1, (0, 255, 0), 2)
            cv2.drawContours(debug_frame[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], cListLeft, -1, (0, 255, 0), 2)
            cv2.drawContours(debug_frame[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], cListRight, -1, (0, 255, 0), 2)
            status = f"Angle: {angle} | Turns: {t} | L: {leftArea} | R: {rightArea}"
            cv2.putText(debug_frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Debug View", debug_frame)

        if arduino:
            try:
                arduino.write(f"{angle}\n".encode())
            except:
                print("⚠️ Error writing to Arduino.")

        prevDiff = aDiff
        prevAngle = angle

        if t >= 12 and abs(angle - straightConst) <= 20:
            print("✅ Lap completed!")
            break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

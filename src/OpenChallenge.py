import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2
from datetime import datetime
import argparse


# debug flag parsing
parser = argparse.ArgumentParser()
parser.add_argument("--debug", action="store_true", help="Enable debug mode")
args = parser.parse_args()
DEBUG = True
#DEBUG = True if args.debug else False
print("DEBUG MODE" if DEBUG else "PRODUCTION")

# Simulated camera settings
CAM_WIDTH = 640
CAM_HEIGHT = 480

# Region of Interest coordinates
ROI1 = [20, 220, 240, 260]
ROI2 = [400, 220, 620, 260]
ROI3 = [200, 300, 440, 350]

# Color ranges
LOWER_BLACK = np.array([21, 109, 112])
UPPER_BLACK = np.array([81, 149, 152])

LOWER_ORANGE = np.array([108, 126, 63])
UPPER_ORANGE = np.array([168, 166, 103])

LOWER_BLUE = np.array([70, 145, 151])
UPPER_BLUE = np.array([130, 185, 191])

# Control parameters
kp = 0.01
kd = 0.0006
straightConst = 95
turnThresh = 150
exitThresh = 1500
tDeviation = 25
sharpRight = straightConst + tDeviation
sharpLeft = straightConst - tDeviation
maxRight = straightConst + 30
maxLeft = straightConst - 30
# Stopping logic
stopFlag = False
stopTime = 0
IntersectionTurningDuration = 2000
IntersectionTurningStart = 0

# Serial communication
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, dsrdtr=True)
time.sleep(3)
arduino.write(b'Hello Arduino\n')

def find_contours(frame, lower_color, upper_color, roi):
    x1, y1, x2, y2 = roi
    roi_frame = frame[y1:y2, x1:x2]
    labImg = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2Lab)
    mask = cv2.inRange(labImg, lower_color, upper_color)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # shift contours back to original image coordinates
    shifted_contours = []
    for cnt in contours:
        cnt = cnt + [x1, y1]   # add ROI offset
        shifted_contours.append(cnt)

    return shifted_contours

def max_contour(contours):
    if len(contours) == 0:
        return (0, None)
    largest = max(contours, key=cv2.contourArea)
    return (cv2.contourArea(largest), largest)

def display_roi(frame, rois, color):
    for roi in rois:
        x1, y1, x2, y2 = roi
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    return frame

def main():
    global stopFlag
    global stopTime
    # Initialize PiCamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    #cap = cv2.VideoCapture(0)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    time.sleep(2)  # Allow camera to warm up

    # State variables
    lTurn = rTurn = lDetected = False
    t = angle = prevAngle = aDiff = prevDiff = 0
    turnDir = "none"

    while True:
        #frame = cap.read()
        frame = picam2.capture_array()
        #frame = cv2.flip(frame, 1)

        # Contour detection
        cListLeft = find_contours(frame, LOWER_BLACK, UPPER_BLACK, ROI1)
        cListRight = find_contours(frame, LOWER_BLACK, UPPER_BLACK, ROI2)
        cListOrange = find_contours(frame, LOWER_ORANGE, UPPER_ORANGE, ROI3)
        cListBlue = find_contours(frame, LOWER_BLUE, UPPER_BLUE, ROI3)

        leftArea, _ = max_contour(cListLeft)
        rightArea, _ = max_contour(cListRight)
        orangeArea, _ = max_contour(cListOrange)
        blueArea, _ = max_contour(cListBlue)

        # Marker detection
        if orangeArea > 100:
            lDetected = True
            if turnDir == "none":
                turnDir = "right"
                print(turnDir)
        elif blueArea > 100:
            lDetected = True
            if turnDir == "none":
                turnDir = "left"
                print(turnDir)

        # PD controller
        aDiff = leftArea - rightArea
        angle = int(max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0))

        if leftArea <= turnThresh and not rTurn:
            lTurn = True
        elif rightArea <= turnThresh and not lTurn:
            rTurn = True

        if (rightArea > exitThresh and rTurn) or (leftArea > exitThresh and lTurn):
            lTurn = rTurn = False
            prevDiff = 0
            if lDetected:
                t += 1
                lDetected = False

        # Intersection turning
        if turnDir == "left":
            angle = maxLeft
        elif turnDir == "right":
            angle = maxRight

        if turnDir == "left" or turnDir == "right":
            turnDir = "none"
            IntersectionTurningStart = datetime.now().timestamp() * 1000  # current time in milliseconds

            while True:
                arduino.write(f"{angle}\n".encode())
                if (datetime.now().timestamp() * 1000 - IntersectionTurningStart) > IntersectionTurningDuration:
                    break

        # Clamp angle
        # if lTurn:
        #    angle = min(max(angle, sharpLeft), maxLeft)
        # elif rTurn:
        #    angle = max(min(angle, sharpRight), maxRight)
        # else:
        #    angle = max(min(angle, sharpLeft), sharpRight)

        if DEBUG:
            debug_frame = frame.copy()
            debug_frame = display_roi(debug_frame, [ROI1, ROI2, ROI3], (255, 0, 255))
            cv2.drawContours(debug_frame[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], cListLeft, -1, (0, 255, 0), 2)
            cv2.drawContours(debug_frame[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], cListRight, -1, (0, 255, 0), 2)
            cv2.drawContours(debug_frame[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cListOrange, -1, (0, 165, 255), 2)
            cv2.drawContours(debug_frame[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cListBlue, -1, (0, 165, 255), 2)
            status = f"Angle: {angle} | Turns: {t} | L: {leftArea} | R: {rightArea}"
            cv2.putText(debug_frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("Debug View", debug_frame)

        # Send to Arduino
        arduino.write(f"{angle}\n".encode())

        prevDiff = aDiff
        prevAngle = angle

        if stopFlag and (datetime.now().microsecond - stopTime) > 50000:
            print("Lap completed!")
            print(angle)
            break

        if t >= 12 and abs(angle - straightConst) <= 15 and not stopFlag:
            stopFlag = True
            stopTime = datetime.now().microsecond

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
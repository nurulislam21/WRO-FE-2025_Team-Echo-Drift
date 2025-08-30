import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2
from datetime import datetime
import sys
import threading
from queue import Queue, Empty
import copy

# debug flag parsing
DEBUG = True
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

LOWER_ORANGE = np.array([166, 120, 84])
UPPER_ORANGE = np.array([226, 160, 124])

LOWER_BLUE = np.array([94, 167, 169])
UPPER_BLUE = np.array([154, 207, 209])

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

slightRight = straightConst + 20
slightLeft = straightConst - 20

# Stopping logic
stopFlag = False
stopTime = 0

# Intersection turning
IntersectionTurningDuration = 1700
IntersectionTurningStart = 0
IntersectionDetected = False

# Serial communication
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, dsrdtr=True)
time.sleep(3)
arduino.write(b'Hello Arduino\n')

# Threading variables
frame_queue = Queue(maxsize=2)  # Small queue to prevent lag
result_queue = Queue(maxsize=2)
processing_thread = None
stop_processing = threading.Event()

class ProcessingResult:
    def __init__(self, leftArea=0, rightArea=0, orangeArea=0, blueArea=0, 
                 contours_left=None, contours_right=None, contours_orange=None, contours_blue=None):
        self.leftArea = leftArea
        self.rightArea = rightArea
        self.orangeArea = orangeArea
        self.blueArea = blueArea
        self.contours_left = contours_left or []
        self.contours_right = contours_right or []
        self.contours_orange = contours_orange or []
        self.contours_blue = contours_blue or []

def find_contours(frame, lower_color, upper_color, roi):
    x1, y1, x2, y2 = roi
    roi_frame = frame[y1:y2, x1:x2]
    labImg = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2Lab)
    img_blur = cv2.GaussianBlur(labImg, (7, 7), 0)
    mask = cv2.inRange(img_blur, lower_color, upper_color)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def max_contour(contours):
    if len(contours) == 0:
        return (0, None)
    largest = max(contours, key=cv2.contourArea)
    return (cv2.contourArea(largest), largest)

def processing_worker():
    """Worker thread for processing frames"""
    while not stop_processing.is_set():
        try:
            # Get frame from queue with timeout
            frame = frame_queue.get(timeout=0.1)
            
            # Process contours
            cListLeft = find_contours(frame, LOWER_BLACK, UPPER_BLACK, ROI1)
            cListRight = find_contours(frame, LOWER_BLACK, UPPER_BLACK, ROI2)
            cListOrange = find_contours(frame, LOWER_ORANGE, UPPER_ORANGE, ROI3)
            cListBlue = find_contours(frame, LOWER_BLUE, UPPER_BLUE, ROI3)

            leftArea, _ = max_contour(cListLeft)
            rightArea, _ = max_contour(cListRight)
            orangeArea, _ = max_contour(cListOrange)
            blueArea, _ = max_contour(cListBlue)

            # Create result object
            result = ProcessingResult(
                leftArea, rightArea, orangeArea, blueArea,
                cListLeft, cListRight, cListOrange, cListBlue
            )

            # Put result in queue (non-blocking, drop old results if queue is full)
            try:
                result_queue.put_nowait(result)
            except:
                # Queue is full, remove old result and add new one
                try:
                    result_queue.get_nowait()
                    result_queue.put_nowait(result)
                except Empty:
                    pass

            frame_queue.task_done()

        except Empty:
            # No frame available, continue
            continue
        except Exception as e:
            print(f"Processing error: {e}")
            continue

def display_roi(frame, rois, color):
    for roi in rois:
        x1, y1, x2, y2 = roi
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    return frame

def main():
    global stopFlag
    global stopTime
    global IntersectionDetected
    global IntersectionTurningStart
    global processing_thread

    # Initialize PiCamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    picam2.set_controls({
        "ExposureTime": 16000,
        "AnalogueGain": 42.0,
        "AeEnable": False,
        "AwbEnable": False,
        "FrameDurationLimits": (40000, 40000)
    })
    picam2.start()

    # Start processing thread
    processing_thread = threading.Thread(target=processing_worker, daemon=True)
    processing_thread.start()

    time.sleep(2)  # Allow camera to warm up

    # State variables
    lTurn = rTurn = lDetected = False
    t = angle = prevAngle = aDiff = prevDiff = 0
    turnDir = "none"
    
    # Initialize with default values
    current_result = ProcessingResult()

    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()

            # Add frame to processing queue (non-blocking)
            try:
                frame_queue.put_nowait(copy.deepcopy(frame))
            except:
                # Queue is full, skip this frame
                pass

            # Get latest processing result (non-blocking)
            try:
                while not result_queue.empty():
                    current_result = result_queue.get_nowait()
            except Empty:
                pass

            # Use the latest processing results
            leftArea = current_result.leftArea
            rightArea = current_result.rightArea
            orangeArea = current_result.orangeArea
            blueArea = current_result.blueArea

            # Marker detection
            if not IntersectionDetected:
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
                angle = slightLeft
            elif turnDir == "right":
                angle = slightRight
            else:
                angle = int(max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0))

            # trigger only once when intersection detected
            if (turnDir == "left" or turnDir == "right") and not IntersectionDetected:
                IntersectionDetected = True
                IntersectionTurningStart = datetime.now().timestamp() * 1000

            if (datetime.now().timestamp() * 1000 - IntersectionTurningStart) > IntersectionTurningDuration and IntersectionDetected:
                turnDir = "none"
                IntersectionDetected = False

            if DEBUG:
                debug_frame = frame.copy()
                debug_frame = display_roi(debug_frame, [ROI1, ROI2, ROI3], (255, 0, 255))
                
                # Draw contours using the latest results
                if current_result.contours_left:
                    cv2.drawContours(debug_frame[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], current_result.contours_left, -1, (0, 255, 0), 2)
                if current_result.contours_right:
                    cv2.drawContours(debug_frame[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]], current_result.contours_right, -1, (0, 255, 0), 2)
                if current_result.contours_orange:
                    cv2.drawContours(debug_frame[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], current_result.contours_orange, -1, (0, 165, 255), 2)
                if current_result.contours_blue:
                    cv2.drawContours(debug_frame[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], current_result.contours_blue, -1, (0, 165, 255), 2)
                
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

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # Cleanup
        stop_processing.set()
        if processing_thread:
            processing_thread.join(timeout=1.0)
        picam2.stop()
        arduino.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

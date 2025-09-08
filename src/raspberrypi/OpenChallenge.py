import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2
from datetime import datetime
import sys
import threading
from queue import Queue, Empty
from img_processing_functions import display_roi
from contour_workers import ContourWorkers, ContourResult
import copy

# debug flag parsing
debug_flag = sys.argv[1] == "--debug" if len(sys.argv) > 1 else ""
if debug_flag:
    DEBUG = True
else:
    DEBUG = False

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

LOWER_ORANGE = np.array([105, 125, 87])
UPPER_ORANGE = np.array([185, 165, 127])

LOWER_BLUE = np.array([92, 150, 166])
UPPER_BLUE = np.array([152, 190, 206])

contour_workers = ContourWorkers(
    lower_blue=LOWER_BLUE,
    upper_blue=UPPER_BLUE,
    lower_black=LOWER_BLACK,
    upper_black=UPPER_BLACK,
    lower_orange=LOWER_ORANGE,
    upper_orange=UPPER_ORANGE,
    roi1=ROI1,
    roi2=ROI2,
    roi3=ROI3,
)

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

startProcessing = False
stopProcessing = False

# Stopping logic
stopFlag = False
stopTime = 0

# Intersection turning
IntersectionTurningDuration = 1700
IntersectionTurningStart = 0
IntersectionDetected = False

# Serial communication
arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, dsrdtr=True)
time.sleep(3)
arduino.write(b"0,95\n")

# Threading variables - separate queues for each detection task
def main():
    global stopFlag
    global stopTime
    global IntersectionDetected
    global IntersectionTurningStart

    # Initialize PiCamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
    picam2.configure(config)
    picam2.set_controls(
        {
            "ExposureTime": 16000,
            "AnalogueGain": 42.0,
            "AeEnable": False,
            "AwbEnable": False,
            "FrameDurationLimits": (40000, 40000),
        }
    )
    picam2.start()

    # Start all processing threads
    threads = []
    workers = [
        contour_workers.left_contour_worker,
        contour_workers.right_contour_worker,
        contour_workers.orange_contour_worker,
        contour_workers.blue_contour_worker,
    ]

    for worker in workers:
        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        threads.append(thread)

    print(f"Started {len(threads)} processing threads")

    time.sleep(2)  # Allow camera to warm up

    # State variables
    lTurn = rTurn = lDetected = False
    t = angle = prevAngle = aDiff = prevDiff = 0
    turnDir = "none"

    # Initialize with default values
    left_result = ContourResult()
    right_result = ContourResult()
    orange_result = ContourResult()
    blue_result = ContourResult()

    try:
        while True:
            # dont start until start button pressed
            if arduino.in_waiting > 0 and not startProcessing:
                line = arduino.readline().decode("utf-8").rstrip()
                print(f"Arduino: {line}")
                if not line == "START":
                    startProcessing = True
                    continue

            # Capture frame
            frame = picam2.capture_array()

            # Distribute frame to all processing threads (non-blocking)
            frame_copy = copy.deepcopy(frame)

            try:
                contour_workers.frame_queue_left.put_nowait(frame_copy)
            except:
                pass  # Skip if queue full

            try:
                contour_workers.frame_queue_right.put_nowait(frame_copy)
            except:
                pass

            try:
                contour_workers.frame_queue_orange.put_nowait(frame_copy)
            except:
                pass

            try:
                contour_workers.frame_queue_blue.put_nowait(frame_copy)
            except:
                pass

            # Collect latest results from all threads (non-blocking)
            try:
                while not contour_workers.result_queue_left.empty():
                    left_result = contour_workers.result_queue_left.get_nowait()
            except Empty:
                pass

            try:
                while not contour_workers.result_queue_right.empty():
                    right_result = contour_workers.result_queue_right.get_nowait()
            except Empty:
                pass

            try:
                while not contour_workers.result_queue_orange.empty():
                    orange_result = contour_workers.result_queue_orange.get_nowait()
            except Empty:
                pass

            try:
                while not contour_workers.result_queue_blue.empty():
                    blue_result = contour_workers.result_queue_blue.get_nowait()
            except Empty:
                pass

            # Use the latest processing results
            leftArea = left_result.area
            rightArea = right_result.area
            orangeArea = orange_result.area
            blueArea = blue_result.area

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
                angle = int(
                    max(straightConst + aDiff * kp + (aDiff - prevDiff) * kd, 0)
                )

            # trigger only once when intersection detected
            if (turnDir == "left" or turnDir == "right") and not IntersectionDetected:
                IntersectionDetected = True
                IntersectionTurningStart = datetime.now().timestamp() * 1000

            if (
                datetime.now().timestamp() * 1000 - IntersectionTurningStart
            ) > IntersectionTurningDuration and IntersectionDetected:
                turnDir = "none"
                IntersectionDetected = False

            if DEBUG:
                debug_frame = frame.copy()
                debug_frame = display_roi(
                    debug_frame, [ROI1, ROI2, ROI3], (255, 0, 255)
                )

                # Draw contours using the latest results
                if left_result.contours:
                    cv2.drawContours(
                        debug_frame[ROI1[1] : ROI1[3], ROI1[0] : ROI1[2]],
                        left_result.contours,
                        -1,
                        (0, 255, 0),
                        2,
                    )
                if right_result.contours:
                    cv2.drawContours(
                        debug_frame[ROI2[1] : ROI2[3], ROI2[0] : ROI2[2]],
                        right_result.contours,
                        -1,
                        (0, 255, 0),
                        2,
                    )
                if orange_result.contours:
                    cv2.drawContours(
                        debug_frame[ROI3[1] : ROI3[3], ROI3[0] : ROI3[2]],
                        orange_result.contours,
                        -1,
                        (0, 165, 255),
                        2,
                    )
                if blue_result.contours:
                    cv2.drawContours(
                        debug_frame[ROI3[1] : ROI3[3], ROI3[0] : ROI3[2]],
                        blue_result.contours,
                        -1,
                        (0, 165, 255),
                        2,
                    )

                status = f"Angle: {angle} | Turns: {t} | L: {leftArea} | R: {rightArea}"
                cv2.putText(
                    debug_frame,
                    status,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
                cv2.imshow("Debug View", debug_frame)

            # Send to Arduino
            arduino.write(f"60,{angle}\n".encode())

            prevDiff = aDiff
            prevAngle = angle

            if stopFlag and (datetime.now().microsecond - stopTime) > 50000:
                print("Lap completed!")
                print(angle)
                break

            if t >= 12 and abs(angle - straightConst) <= 15 and not stopFlag:
                stopFlag = True
                stopTime = datetime.now().microsecond

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # Cleanup
        contour_workers.stop_processing.set()
        for thread in threads:
            thread.join(timeout=1.0)
        picam2.stop()
        arduino.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

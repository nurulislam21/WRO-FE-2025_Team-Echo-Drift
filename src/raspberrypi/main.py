import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2
from datetime import datetime
import sys
import threading
from queue import Queue, Empty
from img_processing_functions import display_roi, get_min_y, get_avg_x
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
ROI1 = [20, 220, 240, 260]  # left
ROI2 = [400, 220, 620, 260]  # right
ROI3 = [200, 300, 440, 350]  # lap detection
ROI4 = [90, 175, 540, 280]  # obstacle detection

# Color ranges
LOWER_BLACK = np.array([21, 109, 112])
UPPER_BLACK = np.array([81, 149, 152])

LOWER_ORANGE = np.array([105, 125, 87])
UPPER_ORANGE = np.array([185, 165, 127])

LOWER_BLUE = np.array([92, 150, 166])
UPPER_BLUE = np.array([152, 190, 206])

# obstacle color ranges
LOWER_RED = np.array([33, 137, 70])
UPPER_RED = np.array([93, 177, 110])

LOWER_GREEN = np.array([60, 88, 150])
UPPER_GREEN = np.array([120, 128, 190])

contour_workers = ContourWorkers(
    lower_blue=LOWER_BLUE,
    upper_blue=UPPER_BLUE,
    lower_black=LOWER_BLACK,
    upper_black=UPPER_BLACK,
    lower_orange=LOWER_ORANGE,
    upper_orange=UPPER_ORANGE,
    lower_red=LOWER_RED,
    upper_red=UPPER_RED,
    lower_green=LOWER_GREEN,
    upper_green=UPPER_GREEN,
    roi1=ROI1,
    roi2=ROI2,
    roi3=ROI3,
    roi4=ROI4,
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
    global IntersectionTurningStart, startProcessing

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
        contour_workers.green_contour_worker,
        contour_workers.red_contour_worker,
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

    try:
        while True:
            # dont start until start button pressed
            if not startProcessing:
                if arduino.in_waiting > 0:
                    line = arduino.readline().decode("utf-8").rstrip()
                    print(f"Arduino: {line}")
                    if not line == "START":
                        startProcessing = True
                        continue

            # Capture frame
            frame = picam2.capture_array()

            # Distribute frame to all processing threads (non-blocking)
            frame_copy = copy.deepcopy(frame)

            contour_workers.put_frames_in_queues(frame_copy)
            # Retrieve all results from queues (non-blocking)
            (
                left_result,
                right_result,
                orange_result,
                blue_result,
                green_result,
                red_result,
            ) = contour_workers.collect_results()

            # Use the latest processing results
            leftArea = left_result.area
            rightArea = right_result.area
            orangeArea = orange_result.area
            blueArea = blue_result.area
            greenArea = green_result.area
            redArea = red_result.area

            # Marker detection
            if not IntersectionDetected:
                if orangeArea > 80:
                    lDetected = True
                    if turnDir == "none":
                        turnDir = "right"
                        print(turnDir)
                elif blueArea > 80:
                    lDetected = True
                    if turnDir == "none":
                        turnDir = "left"
                        print(turnDir)

            # overwrite leftArea/rightArea with obstacle areas if detected
            if greenArea > 100 or redArea > 100:
                # get the nearer obstacle
                green_piller_y_distance = get_min_y(green_result.contours)
                red_piller_y_distance = get_min_y(red_result.contours)

                green_piller_y_distance = float("inf") if green_piller_y_distance is None else green_piller_y_distance
                red_piller_y_distance = float("inf") if red_piller_y_distance is None else red_piller_y_distance

                if green_piller_y_distance < red_piller_y_distance:
                    print("green piller detected")
                    rightArea = (
                        (CAM_WIDTH - get_avg_x(green_result.contours)) * 2
                    ) / CAM_WIDTH
                elif red_piller_y_distance < green_piller_y_distance:
                    print("red piller detected")
                    leftArea = (get_avg_x(red_result.contours) * 2) / CAM_WIDTH

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
                    debug_frame, [ROI1, ROI2, ROI3, ROI4], (255, 0, 255)
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

                if green_result.contours:
                    cv2.drawContours(
                        debug_frame[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
                        green_result.contours,
                        -1,
                        (0, 255, 0),
                        2,
                    )
                if red_result.contours:
                    cv2.drawContours(
                        debug_frame[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
                        red_result.contours,
                        -1,
                        (0, 0, 255),
                        2,
                    )

                status = f"Angle: {angle} | Turns: {t} | L: {leftArea} | R: {rightArea} | G: {greenArea} | R: {redArea}"
                cv2.putText(
                    debug_frame,
                    status,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
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

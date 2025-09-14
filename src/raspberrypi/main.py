import sys
import time
import serial
import cv2
import numpy as np
import threading
from picamera2 import Picamera2
import time
from collections import deque
from img_processing_functions import display_roi, get_max_y_coord, get_min_x_coord
from contour_workers import ContourWorkers
from simple_pid import PID
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
MAX_SPEED = 30
MIN_SPEED = 30

# Intersections
TOTAL_INTERSECTIONS = 12

# Region of Interest coordinates
ROI1 = [20, 220, 240, 280]  # left
ROI2 = [400, 220, 620, 280]  # right
ROI3 = [200, 300, 440, 350]  # lap detection
ROI4 = [90, 140, 540, 320]  # obstacle detection

BLACK_WALL_DETECTOR_AREA = (ROI1[2] - ROI1[0]) * (ROI1[3] - ROI1[1])
OBSTACLE_DETECTOR_X = ROI4[2] - ROI4[0]
OBSTACLE_DETECTOR_Y = ROI4[3] - ROI4[1]

REVERSE_TRIGGER_Y = OBSTACLE_DETECTOR_Y - 20
REVERSE_TRIGGER_X_MIN = (OBSTACLE_DETECTOR_X // 2) - 150
REVERSE_TRIGGER_X_MAX = (OBSTACLE_DETECTOR_X // 2) + 150

# Color ranges
LOWER_BLACK = np.array([0, 114, 116])
UPPER_BLACK = np.array([58, 154, 156])

LOWER_ORANGE = np.array([105, 125, 87])
UPPER_ORANGE = np.array([185, 165, 127])

LOWER_BLUE = np.array([92, 150, 166])
UPPER_BLUE = np.array([152, 190, 206])

# obstacle color ranges
LOWER_RED = np.array([48, 154, 39])
UPPER_RED = np.array([108, 194, 79])

LOWER_GREEN = np.array([60, 88, 150])
UPPER_GREEN = np.array([120, 128, 190])

contour_workers = ContourWorkers(
    # mode="NO_OBSTACLE",
    mode="OBSTACLE",
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

STRAIGHT_CONST = 95
turnThresh = 150
exitThresh = 1500


MAX_OFFSET_DEGREE = 30
maxRight = STRAIGHT_CONST + MAX_OFFSET_DEGREE
maxLeft = STRAIGHT_CONST - MAX_OFFSET_DEGREE
slightRight = STRAIGHT_CONST + 20
slightLeft = STRAIGHT_CONST - 20

# PID controller constants
kp = 1.5
ki = 0.0
kd = 0.07
pid = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=0)
pid.output_limits = (-1, 1)  # limit output to -1 to 1
pid.sample_time = 0.02
SMOOTH_WINDOW = 3
left_buf = deque(maxlen=SMOOTH_WINDOW)
right_buf = deque(maxlen=SMOOTH_WINDOW)

# Start/Stopping logic
startProcessing = False
stopFlag = False
stopTime = 0

# Intersection crossing
current_intersections = 0
intersection_crossing_duration = 1.1  # seconds
intersection_crossing_start = 0
intersection_detected = False

# Serial communication
arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, dsrdtr=True)
time.sleep(3)
arduino.write(b"0,95\n")


# Threading variables - separate queues for each detection task
def main():
    global stopFlag, stopTime
    global current_intersections, intersection_detected, intersection_crossing_start
    global startProcessing

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
    # lTurn = rTurn = lDetected = False
    # t = 0
    # turnDir = "none"
    angle = STRAIGHT_CONST

    try:
        while True:
            # dont start until start button pressed
            # if arduino.in_waiting > 0:
            #     line = arduino.readline().decode("utf-8").rstrip()
            #     print(f"Arduino: {line}")
            #     if not line == "START":
            #         startProcessing = True
            # if not startProcessing:
            #     continue

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
            left_area = left_result.area
            right_area = right_result.area
            orange_area = orange_result.area
            blue_area = blue_result.area
            green_area = green_result.area
            red_area = red_result.area            

            # intersection detection
            if not intersection_detected:
                if (orange_result.contours and orange_area > 80) or (
                    blue_result.contours and blue_area > 80
                ):
                    intersection_detected = True
                    intersection_crossing_start = int(time.time())
                    current_intersections += 1
                    print(
                        f"Intersection detected! Count: {current_intersections}/{TOTAL_INTERSECTIONS}"
                    )
            else:
                if (
                    int(time.time()) - intersection_crossing_start
                    > intersection_crossing_duration
                ):
                    intersection_detected = False
                    print("Intersection crossing ended.")

            # --- Obstacle avoidance ---
            if contour_workers.mode == "OBSTACLE" and red_result.contours and red_area > 300:
                # pick nearest red object (smallest cy)
                obj_x, obj_y = get_max_y_coord(red_result.contours)
                r_wall_x, r_wall_y = get_min_x_coord(right_result.contours)

                if (r_wall_x is None and r_wall_y is None):
                    print("No wall detected!")
                    # set default wall position if none detected
                    r_wall_x = OBSTACLE_DETECTOR_X
                    r_wall_y = OBSTACLE_DETECTOR_Y // 2

                if not (obj_x is None and obj_y is None):
                    # if object is too close, back off
                    if obj_y > REVERSE_TRIGGER_Y and obj_x > REVERSE_TRIGGER_X_MIN and obj_x < REVERSE_TRIGGER_X_MAX:
                        print("Object too close! Backing off.")
                        for _ in range(7):       
                            arduino.write(f"-{MIN_SPEED},{STRAIGHT_CONST}\n".encode())
                            time.sleep(0.1)
                        
                        for _ in range(5):       
                            arduino.write(f"0,{STRAIGHT_CONST}\n".encode())
                            time.sleep(0.1)                        

                    # transform to global coordinates
                    obj_x += ROI4[0]                    
                    r_wall_x += ROI2[0]              

                    # compute how far is the bot from the object and walls middle point
                    offset_x = (obj_x + ((r_wall_x - obj_x) // 2)) - (OBSTACLE_DETECTOR_X // 2)
                    obj_error = (offset_x / (OBSTACLE_DETECTOR_X // 2))  # normalized [-1, 1]
                    obj_error = - np.clip(obj_error, -1, 1)
                    normalized_angle_offset = pid(obj_error)

                    # steer more aggressively when closer to object
                    y_gain = np.interp(obj_y, [0, OBSTACLE_DETECTOR_Y], [0, 1])
                    normalized_angle_offset *= y_gain
                    print(f"Obj error: {obj_error} | PID output: {normalized_angle_offset} | y_gain: {y_gain}")

                print(f"Obj: {obj_x}, {obj_y} | Wall: {r_wall_x}, {r_wall_y}")
            
            else:
                # PID controller
                left_buf.append(left_area)
                right_buf.append(right_area)
                left_s = sum(left_buf) / len(left_buf)
                right_s = sum(right_buf) / len(right_buf)
                aDiff = right_s - left_s
                aSum = left_s + right_s
                error = aDiff / (aSum + 1e-6)  # normalized between roughly [-1,1]
                normalized_angle_offset = pid(error)
                print(f"Line error: {error}, PID output: {normalized_angle_offset}")

            # --- Map normalized control to servo angle ---
            angle = int(
                max(
                    min(STRAIGHT_CONST + normalized_angle_offset * MAX_OFFSET_DEGREE, maxRight),
                    maxLeft,
                )
            )

            # map speed with angle
            speed = np.interp(
                angle,
                [maxLeft, STRAIGHT_CONST, maxRight],
                [MIN_SPEED, MAX_SPEED, MIN_SPEED],
            )
            # speed = 0

            # Send to Arduino
            arduino.write(f"{speed},{angle}\n".encode())

            if DEBUG:
                debug_frame = frame.copy()
                debug_frame = display_roi(
                    debug_frame, [ROI1, ROI2, ROI3, ROI4], (255, 0, 255)
                )

                # draw reverse trigger zone
                cv2.rectangle(
                    debug_frame,
                    (ROI4[0] + REVERSE_TRIGGER_X_MIN, ROI4[1] + REVERSE_TRIGGER_Y),
                    (ROI4[0] + REVERSE_TRIGGER_X_MAX, ROI4[3]),
                    (255, 0, 0),
                    2,
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

                status = f"Angle: {angle} | Turns: {current_intersections/4} | L: {left_area} | R: {right_area} | OR: {orange_area} | BL: {blue_area}"
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

            if stopFlag and (int(time.time()) - stopTime) > 1.7:
                print("Lap completed!")
                arduino.write(f"0,{angle}\n".encode())
                print(angle)
                break

            if (
                current_intersections >= 12
                # and abs(angle - STRAIGHT_CONST) <= 15
                and not stopFlag
            ):
                stopFlag = True
                stopTime = int(time.time())
                print("Preparing to stop...")

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

import sys
import time
import serial
import cv2
import numpy as np
import threading
from picamera2 import Picamera2
import time
from collections import deque
from img_processing_functions import (
    display_roi,
    get_max_y_coord,
    get_min_x_coord,
    display_debug_screen,
    get_max_x_coord,
    get_overall_centroid
)
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
MAX_SPEED = 60
MIN_SPEED = 45

# Intersections
TOTAL_INTERSECTIONS = 100

# Region of Interest coordinates
ROI1 = [20, 220, 240, 280]  # left
ROI2 = [400, 220, 620, 280]  # right
ROI3 = [200, 300, 440, 350]  # lap detection
ROI4 = [90, 140, 540, 320]  # obstacle detection

BLACK_WALL_DETECTOR_AREA = (ROI1[2] - ROI1[0]) * (ROI1[3] - ROI1[1])
OBSTACLE_DETECTOR_X = ROI4[2] - ROI4[0]
OBSTACLE_DETECTOR_Y = ROI4[3] - ROI4[1]
obstacle_wall_pivot = (None, None)

REVERSE_TRIGGER_Y = OBSTACLE_DETECTOR_Y - 20
REVERSE_TRIGGER_X_MIN = (OBSTACLE_DETECTOR_X // 2) - 115
REVERSE_TRIGGER_X_MAX = (OBSTACLE_DETECTOR_X // 2) + 115

# Color ranges
LOWER_BLACK = np.array([0, 114, 116])
UPPER_BLACK = np.array([65, 154, 156])

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

# Start/reverse/Stopping logic
speed = 0

trigger_reverse = False
reverse_start_time = 0
reverse_duration = 1.0  # seconds

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
    global stopFlag, stopTime, speed, trigger_reverse
    global current_intersections, intersection_detected, intersection_crossing_start
    global startProcessing, obstacle_wall_pivot

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

            # default values
            speed_factor = 1.0

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

            # Debug view
            if DEBUG:
                display_debug_screen(
                    frame=frame,
                    CAM_WIDTH=CAM_WIDTH,
                    CAM_HEIGHT=CAM_HEIGHT,
                    left_result=left_result,
                    right_result=right_result,
                    orange_result=orange_result,
                    blue_result=blue_result,
                    green_result=green_result,
                    red_result=red_result,
                    ROI1=ROI1,
                    ROI2=ROI2,
                    ROI3=ROI3,
                    ROI4=ROI4,
                    REVERSE_TRIGGER_X_MIN=REVERSE_TRIGGER_X_MIN,
                    REVERSE_TRIGGER_X_MAX=REVERSE_TRIGGER_X_MAX,
                    REVERSE_TRIGGER_Y=REVERSE_TRIGGER_Y,
                    angle=angle,
                    current_intersections=current_intersections,
                    left_area=left_area,
                    right_area=right_area,
                    orange_area=orange_area,
                    blue_area=blue_area,
                    obstacle_wall_pivot=obstacle_wall_pivot,
                )

            if trigger_reverse:
                speed = -MIN_SPEED
                if (time.time() - reverse_start_time) > reverse_duration:
                    trigger_reverse = False
                    speed = 0  # stop after reversing
                else:
                    angle = STRAIGHT_CONST  # go straight when reversing
                arduino.write(f"{speed},{angle}\n".encode())
                print(f"Reversing... Speed: {speed}, Angle: {angle}")
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            # --- Obstacle avoidance ---
            if (
                contour_workers.mode == "OBSTACLE"
                and red_result.contours
                and red_area > 300
            ):
                # pick nearest red object (smallest cy)
                obj_x, obj_y = get_max_y_coord(red_result.contours)
                # r_wall_x, r_wall_y = get_min_x_coord(right_result.contours)
                r_wall_x, r_wall_y = get_overall_centroid(right_result.contours)

                if r_wall_x is None:
                    print("No wall detected!")
                    # set default wall position if none detected
                    r_wall_x = CAM_WIDTH
                else:
                    # transform to global coordinates
                    r_wall_x += ROI2[0]

                if not (obj_x is None and obj_y is None):
                    # if object is too close, back off
                    if (
                        obj_y > REVERSE_TRIGGER_Y
                        and obj_x > REVERSE_TRIGGER_X_MIN
                        and obj_x < REVERSE_TRIGGER_X_MAX
                    ):
                        print("Object too close! Backing off.")
                        trigger_reverse = True
                        reverse_start_time = time.time()

                    # transform to global coordinates
                    obj_x += ROI4[0]
                    obj_y += ROI4[1]

                    # compute how far is the bot from the object and walls middle point
                    offset_x = (obj_x + ((r_wall_x - obj_x) // 2)) - (CAM_WIDTH // 2)

                    # show a circle dot on the middle of the object and wall
                    obstacle_wall_pivot = (obj_x + ((r_wall_x - obj_x) // 2), obj_y)                     

                    obj_error = offset_x / (CAM_WIDTH // 2)  # normalized [-1, 1]
                    obj_error = -np.clip(
                        obj_error * 8, -1, 1
                    )  # amplify to make it more responsive
                    normalized_angle_offset = pid(obj_error)

                    # steer more aggressively when closer to object
                    y_gain = np.interp(
                        obj_y,
                        [0, (CAM_HEIGHT // 2) - 50, (CAM_HEIGHT // 2) - 20, ROI4[3]],
                        [0, 0.2, 0.7, 1],
                    )
                    normalized_angle_offset *= y_gain
                    speed_factor = 1 - (0.3 * y_gain)  # slow down when closer to object
                    # print(
                    #     f"Obj error: {obj_error} | PID output: {normalized_angle_offset} | y_gain: {y_gain}"
                    # )
                    # print(
                    #     f"offset_x: {offset_x}, obj_x: {obj_x}, r_wall_x: {r_wall_x}, obj_y: {obj_y}"
                    # )
                    # print(f"Normalized angle offset: {normalized_angle_offset}")

                # print(f"Obj: {obj_x}, {obj_y} | Wall: {r_wall_x}, {r_wall_y}")

            else:
                obstacle_wall_pivot = (None, None)
                # PID controller
                left_buf.append(left_area)
                right_buf.append(right_area)
                left_s = sum(left_buf) / len(left_buf)
                right_s = sum(right_buf) / len(right_buf)
                aDiff = right_s - left_s
                aSum = left_s + right_s
                error = aDiff / (aSum + 1e-6)  # normalized between roughly [-1,1]
                normalized_angle_offset = pid(error)
                # left_x, left_y = get_max_x_coord(left_result.contours)
                # right_x, right_y = get_min_x_coord(right_result.contours)

                # if left_x is None:
                #     left_x = 0
                # else:
                #     # transform to global coordinates
                #     left_x += ROI1[0]

                # if right_x is None:
                #     right_x = CAM_WIDTH
                # else:
                #     # transform to global coordinates
                #     right_x += ROI2[0]

                # offset_x = (left_x + ((right_x - left_x) // 2)) - (CAM_WIDTH // 2)
                # wall_error = offset_x / (CAM_WIDTH // 2)

                # wall_error = -np.clip(
                #     wall_error * 2, -1, 1
                # )  # amplify to make it more responsive
                # normalized_angle_offset = pid(wall_error)

                # print(
                #     f"Line error: {wall_error}, PID output: {normalized_angle_offset}"
                # )

            # --- Map normalized control to servo angle ---
            angle = int(
                max(
                    min(
                        STRAIGHT_CONST + normalized_angle_offset * MAX_OFFSET_DEGREE,
                        maxRight,
                    ),
                    maxLeft,
                )
            )

            # map speed with angle
            speed = speed_factor * np.interp(
                angle,
                [maxLeft, STRAIGHT_CONST, maxRight],
                [MIN_SPEED, MAX_SPEED, MIN_SPEED],
            )
            # speed = 0

            # Send to Arduino
            arduino.write(f"{speed},{angle}\n".encode())

            # intersection detection
            # work here
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

            # Stopping logic
            if stopFlag and (int(time.time()) - stopTime) > 1.7:
                print("Lap completed!")
                arduino.write(f"0,{angle}\n".encode())
                print(angle)
                break

            if (
                current_intersections >= TOTAL_INTERSECTIONS
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

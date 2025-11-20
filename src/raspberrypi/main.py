import os
import json
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
    get_overall_centroid,
    point_position,
)
from contour_workers import ContourWorkers
from parking import Parking
from simple_pid import PID
import copy
import RPi.GPIO as GPIO
from odometry import OdometryTracker, OdometryVisualizer, clamp_angle
import math

# debug flag parsing
debug_flag = sys.argv[1] == "--debug" if len(sys.argv) > 1 else ""
if debug_flag:
    DEBUG = True
else:
    DEBUG = False

BUZZER_PIN = 4
# DEBUG = True
print("DEBUG MODE" if DEBUG else "PRODUCTION")

# Simulated camera settings
MODE = "OBSTACLE"  # "NO_OBSTACLE" or "OBSTACLE"
CAM_WIDTH = 640
CAM_HEIGHT = 480
MAX_SPEED = 50 if MODE == "OBSTACLE" else 90
MIN_SPEED = 40 if MODE == "OBSTACLE" else 50

# Intersections
TOTAL_INTERSECTIONS = 12

# Region of Interest coordinates
LEFT_REGION = (
    [0, 190, 230, 240] if MODE == "NO_OBSTACLE" else [0, 190, 230, 240]
)  # left
RIGHT_REGION = (
    [410, 190, 640, 240] if MODE == "NO_OBSTACLE" else [410, 190, 640, 240]
)  # right
LAP_REGION = [215, 260, 415, 305]  # lap detection
OBS_REGION = [77, 110, 563, 315]  # obstacle detection
REVERSE_REGION = [213, 240, 427, 268]  # reverse trigger area
FRONT_WALL_REGION = [300, 155, 340, 175]  # front wall detection
PARKING_LOT_REGION = [0, 185, CAM_WIDTH, 400]  # parking lot detection
# DANGER_ZONE_POINTS = [175, OBS_REGION[1], 465, OBS_REGION[3]]  # area to check for obstacles
DANGER_ZONE_POINTS = [
    {
        "x1": 267,
        "y1": OBS_REGION[1],
        "x2": 182,
        "y2": OBS_REGION[3],
    },
    {
        "x1": 373,
        "y1": OBS_REGION[1],
        "x2": 458,
        "y2": OBS_REGION[3],
    },
]

BLACK_WALL_DETECTOR_AREA = (LEFT_REGION[2] - LEFT_REGION[0]) * (
    LEFT_REGION[3] - LEFT_REGION[1]
)
OBSTACLE_DETECTOR_X = OBS_REGION[2] - OBS_REGION[0]
OBSTACLE_DETECTOR_Y = OBS_REGION[3] - OBS_REGION[1]
obstacle_wall_pivot = (None, None)

# Color ranges
# Load color ranges from file
color_ranges = {}
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    tools_dir = os.path.join(script_dir, "..", "tools")
    tools_dir = os.path.abspath(tools_dir)
    with open(os.path.join(tools_dir, "color_ranges.json"), "r") as f:
        color_ranges = json.load(f)
except FileNotFoundError:
    print("Color ranges file not found. Using default values.")

# Boundary
LOWER_BLACK = (
    np.array(color_ranges["LOWER_BLACK"])
    if color_ranges["BLACK_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_BLACK_HSV"])
)
UPPER_BLACK = (
    np.array(color_ranges["UPPER_BLACK"])
    if color_ranges["BLACK_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_BLACK_HSV"])
)

# Lane color ranges
LOWER_ORANGE = (
    np.array(color_ranges["LOWER_ORANGE"])
    if color_ranges["ORANGE_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_ORANGE_HSV"])
)
UPPER_ORANGE = (
    np.array(color_ranges["UPPER_ORANGE"])
    if color_ranges["ORANGE_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_ORANGE_HSV"])
)
LOWER_BLUE = (
    np.array(color_ranges["LOWER_BLUE"])
    if color_ranges["BLUE_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_BLUE_HSV"])
)
UPPER_BLUE = (
    np.array(color_ranges["UPPER_BLUE"])
    if color_ranges["BLUE_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_BLUE_HSV"])
)

# Obstacle color ranges
LOWER_RED = (
    np.array(color_ranges["LOWER_RED"])
    if color_ranges["RED_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_RED_HSV"])
)
UPPER_RED = (
    np.array(color_ranges["UPPER_RED"])
    if color_ranges["RED_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_RED_HSV"])
)
LOWER_GREEN = (
    np.array(color_ranges["LOWER_GREEN"])
    if color_ranges["GREEN_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_GREEN_HSV"])
)
UPPER_GREEN = (
    np.array(color_ranges["UPPER_GREEN"])
    if color_ranges["GREEN_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_GREEN_HSV"])
)

# Parking color ranges
LOWER_MAGENTA = (
    np.array(color_ranges["LOWER_MAGENTA"])
    if color_ranges["MAGENTA_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["LOWER_MAGENTA_HSV"])
)
UPPER_MAGENTA = (
    np.array(color_ranges["UPPER_MAGENTA"])
    if color_ranges["MAGENTA_COLOR_SPACE"] == "LAB"
    else np.array(color_ranges["UPPER_MAGENTA_HSV"])
)

# reverse_black
LOWER_REVERSE_BLACK = LOWER_BLACK
UPPER_REVERSE_BLACK = UPPER_BLACK

contour_workers = ContourWorkers(
    # mode="NO_OBSTACLE",
    color_ranges=color_ranges,
    mode=MODE,
    has_parked_out=False,
    # color ranges
    lower_blue=LOWER_BLUE,
    upper_blue=UPPER_BLUE,
    lower_black=LOWER_BLACK,
    upper_black=UPPER_BLACK,
    lower_reverse_black=LOWER_REVERSE_BLACK,
    upper_reverse_black=UPPER_REVERSE_BLACK,
    lower_orange=LOWER_ORANGE,
    upper_orange=UPPER_ORANGE,
    lower_red=LOWER_RED,
    upper_red=UPPER_RED,
    lower_green=LOWER_GREEN,
    upper_green=UPPER_GREEN,
    upper_magenta=UPPER_MAGENTA,
    lower_magenta=LOWER_MAGENTA,
    # regions
    left_region=LEFT_REGION,
    right_region=RIGHT_REGION,
    lap_region=LAP_REGION,
    obs_region=OBS_REGION,
    front_wall_region=FRONT_WALL_REGION,
    reverse_region=REVERSE_REGION,
    parking_lot_region=PARKING_LOT_REGION,
)

contour_workers.parking_mode = False

STRAIGHT_CONST = 95
turnThresh = 150
exitThresh = 1500

MAX_OFFSET_DEGREE = 55
maxRight = STRAIGHT_CONST + MAX_OFFSET_DEGREE
maxLeft = STRAIGHT_CONST - MAX_OFFSET_DEGREE

# PID controller constants
kp = 1.75 if MODE == "OBSTACLE" else 1.6
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
reverse_duration = 0.6  # seconds
stop_timer = 2.4
reverse_angle = STRAIGHT_CONST
reverse_pause_time = (
    1.3  # seconds to wait after reversing, will not initiate reverse during this period
)
last_reverse_end_time = 0

startProcessing = False
stopFlag = False
stopTime = 0

# Intersection crossing
current_intersections = 0
intersection_crossing_duration = (
    1.1 if MODE == "NO_OBSTACLE" else 1.5
)  # longer for obstacle mode
intersection_crossing_start = 0
intersection_detected = False

# Serial communication
arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, dsrdtr=True)
time.sleep(2)
arduino.write(b"0,-1,95\n")

# parking
parking = Parking(
    parking_speed=35,
    arduino=arduino,
    camera_width=CAM_WIDTH,
    camera_height=CAM_HEIGHT,
    parking_lot_region=PARKING_LOT_REGION,
    maxLeft=maxLeft,
    maxRight=maxRight,
    STRAIGHT_CONST=STRAIGHT_CONST,
    MAX_OFFSET_DEGREE=MAX_OFFSET_DEGREE,
    REVERSE_REGION=REVERSE_REGION,
)
parking.has_parked_out = False


# Odometry
# init odometry tracker and visualizer
start_zone_rect = [0.75, 1.5]  # meters x, y
tracker = OdometryTracker(
    wheel_radius=0.046, ticks_per_rev=2220, gear_ratio=1.0, debug=DEBUG
)
visualizer = OdometryVisualizer(
    title="Odometry Path (Single Encoder + Gyro)",
    start_zone_rect=start_zone_rect,
    debug=DEBUG,
)

encoder_ticks = 0
gyro_angle = 0.0
prev_encoder_ticks = 0
prev_gyro_angle = 0.0
last_odometry_time = time.time()
last_lap_time = time.time()
current_lap = 0
odometry_lap_samples = {}
DRIFT_ALPHA = 0.3  # between 0.05–0.3, depending on how noisy your drift is
MAX_DRIFT_THRESHOLD = 0.5
LAP_COUNT_INTERVAL = 6  # seconds
smoothed_drift_x = 0.0
smoothed_drift_y = 0.0


# Threading variables - separate queues for each detection task
def main():
    global stopFlag, stopTime, speed, trigger_reverse, reverse_angle, last_reverse_end_time
    global current_intersections, intersection_detected, intersection_crossing_start
    global startProcessing, obstacle_wall_pivot, reverse_start_time
    global encoder_ticks, gyro_angle, prev_encoder_ticks, prev_gyro_angle
    global last_odometry_time, last_lap_time, current_lap, odometry_lap_samples
    global smoothed_drift_x, smoothed_drift_y

    # parking_walls = []
    # parking_walls_count = 0
    # parking_wall_pivot = (None, None)

    # obstacle position
    red_obj_x, red_obj_y = None, None
    green_obj_x, green_obj_y = None, None
    # show_front_wall = False

    # Initialize PiCamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (CAM_WIDTH, CAM_HEIGHT)}
    )
    picam2.configure(config)
    # load camera settings from file
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        tools_dir = os.path.join(script_dir, "..", "tools")
        tools_dir = os.path.abspath(tools_dir)
        with open(os.path.join(tools_dir, "camera_settings.json"), "r") as f:
            settings = json.load(f)
            picam2.set_controls(settings)
            print("Loaded camera settings from file.")
    except FileNotFoundError:
        # Default settings if no file found
        print("No camera settings file found. Using default settings.")
        return

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
        contour_workers.reverse_n_front_wall_contour_worker,
        contour_workers.parking_lot_contour_worker,
    ]

    for worker in workers:
        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        threads.append(thread)

    print(f"Started {len(threads)} processing threads")

    time.sleep(2)  # Allow camera to warm up

    # RPIO setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)

    # buzz to indicate ready
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    time.sleep(0.3)
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    angle = STRAIGHT_CONST

    try:
        while True:
            # dont start until start button pressed
            if arduino.in_waiting > 0:
                line = arduino.readline().decode("utf-8").rstrip()
                print(f"Arduino: {line}")
                if not startProcessing and line == "START":
                    startProcessing = True
                    last_lap_time = time.time()

                else:
                    # Get gyro data and encoder ticks
                    parts = line.split(",")
                    if len(parts) != 2:
                        continue
                    try:
                        encoder_ticks = -int(parts[0])
                        gyro_angle = float(parts[1])
                        print(f"Ticks: {encoder_ticks}, Gyro Angle: {gyro_angle}")
                    except ValueError:
                        continue

            # draw odometry
            if (
                (time.time() - last_odometry_time) >= 0.2
                and (abs(encoder_ticks - prev_encoder_ticks) > 300)
                # dont collect odometry data during parking maneuver
                and (parking.has_parked_out if MODE == "OBSTACLE" else True)
            ):
                # gyro_angle = clamp_angle(gyro_angle, threshold=7)
                # Update odometry tracker
                tracker.update(encoder_ticks, gyro_angle)
                # Get current position
                x, y, theta = tracker.get_position()
                # Update previous values
                prev_encoder_ticks = encoder_ticks
                prev_gyro_angle = gyro_angle
                last_odometry_time = time.time()

                if odometry_lap_samples.get(current_lap) is None:
                    odometry_lap_samples[current_lap] = []
                odometry_lap_samples[current_lap].append((x, y))

                # Drift correction using previous lap data
                if current_lap > 0:
                    # skip correction for first few samples
                    if len(odometry_lap_samples[current_lap]) > 6:
                        # get closest point to (x, y) from previous lap, because robot starts from a different position
                        prev_lap_points = odometry_lap_samples[current_lap - 1]
                        closest_point = min(
                            prev_lap_points,
                            key=lambda p: math.sqrt((p[0] - x) ** 2 + (p[1] - y) ** 2),
                        )
                        # Raw drift measurement
                        new_drift_x = x - closest_point[0]
                        new_drift_y = y - closest_point[1]
                        drift_mag = math.sqrt(new_drift_x**2 + new_drift_y**2)
                        if drift_mag < MAX_DRIFT_THRESHOLD:

                            # Smoothed drift (exponential moving average)
                            smoothed_drift_x = (
                                DRIFT_ALPHA * new_drift_x
                                + (1 - DRIFT_ALPHA) * smoothed_drift_x
                            )
                            smoothed_drift_y = (
                                DRIFT_ALPHA * new_drift_y
                                + (1 - DRIFT_ALPHA) * smoothed_drift_y
                            )

                            # Apply partial correction
                            correction_rate = 0.25
                            tracker.x -= smoothed_drift_x * correction_rate
                            tracker.y -= smoothed_drift_y * correction_rate

                # Update visualization
                visualizer.update_plot(tracker.get_position_history())

                # current_intersections = round(abs(gyro_angle) / 90)
                if abs(x) < start_zone_rect[0] and abs(y) < start_zone_rect[1]:
                    if (time.time() - last_lap_time) >= LAP_COUNT_INTERVAL:
                        current_lap += 1
                        current_intersections += 4

                    # keep resetting lap time only when inside start zone
                    last_lap_time = time.time()

                print(
                    f"Position: x={x:.3f}m, y={y:.3f}m, θ={math.degrees(theta):.1f}° | Intersections: {current_intersections}"
                )

            # Capture frame
            frame = picam2.capture_array()

            # Distribute frame to all processing threads (non-blocking)
            frame_copy = frame.copy()
            contour_workers.put_frames_in_queues(frame_copy)

            # default values
            speed_factor = 1.0

            # Retrieve all results from queues (non-blocking)
            (
                left_result,
                right_result,
                orange_result,
                blue_result,
                green_result,
                red_result,
                reverse_result,
                front_wall_result,
                parking_result,
            ) = contour_workers.collect_results()

            # Use the latest processing results
            left_area = left_result.area
            right_area = right_result.area
            orange_area = orange_result.area
            blue_area = blue_result.area
            green_area = green_result.area
            red_area = red_result.area
            reverse_area = reverse_result.area
            front_wall_area = front_wall_result.area

            # Debug view
            if DEBUG:
                display_debug_screen(
                    mode=MODE,
                    frame=frame,
                    CAM_WIDTH=CAM_WIDTH,
                    CAM_HEIGHT=CAM_HEIGHT,
                    left_result=left_result,
                    right_result=right_result,
                    orange_result=orange_result,
                    blue_result=blue_result,
                    green_result=green_result,
                    red_result=red_result,
                    reverse_result=reverse_result,
                    LEFT_REGION=LEFT_REGION,
                    RIGHT_REGION=RIGHT_REGION,
                    LAP_REGION=LAP_REGION,
                    OBS_REGION=OBS_REGION,
                    REVERSE_REGION=REVERSE_REGION,
                    FRONT_WALL_REGION=FRONT_WALL_REGION,
                    # show_front_wall=show_front_wall,
                    DANGER_ZONE_POINTS=DANGER_ZONE_POINTS,
                    front_wall_result=front_wall_result,
                    angle=angle,
                    current_intersections=current_intersections,
                    left_area=left_area,
                    right_area=right_area,
                    obstacle_wall_pivot=obstacle_wall_pivot,
                    parking_mode=contour_workers.parking_mode,
                    parking_lot_region=PARKING_LOT_REGION,
                    parking_result=parking_result,
                    gyro_angle=gyro_angle,
                )

            if not startProcessing:
                if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                # Dont start moving until start signal received
                continue

            # --- PARKING OUT LOGIC ---
            if contour_workers.mode == "OBSTACLE":
                # process parking out first if not yet done
                if not parking.has_parked_out:
                    parking.process_parking_out(
                        left_result=left_result, right_result=right_result
                    )
                    parking.has_parked_out = contour_workers.has_parked_out = True

                    # set dir to odometry visualizer
                    visualizer.set_dir(
                        "ccw" if parking.parking_lot_side == "left" else "cw"
                    )
                    # collect parking out odometry history and update tracker
                    # for entry in parking.parking_out_odometry_history:
                    #     tracker.update(entry[0], entry[1])
                    # set the lap time to now to avoid lap count right after parking
                    last_lap_time = time.time()
                    continue

            # --- Reversing logic ---
            if (
                trigger_reverse
                # avoid reversing again too soon after last reverse
                and (last_reverse_end_time + reverse_pause_time) < time.time()
            ):
                speed = -MIN_SPEED
                if (time.time() - reverse_start_time) > reverse_duration:
                    trigger_reverse = False
                    last_reverse_end_time = time.time()
                    speed = 20  # stop after reversing
                    reverse_angle = STRAIGHT_CONST
                arduino.write(f"{speed},-1,{reverse_angle}\n".encode())
                print(f"Reversing... Speed: {speed}, Angle: {reverse_angle}")

                # stop timer reset
                if stopFlag:
                    stopTime = int(time.time())

                if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                print("continue")
                continue

            elif reverse_area > 1000 and not (
                # avoid triggering reverse when front wall is too close in obstacle mode
                # cause we want to reverse out in angle
                front_wall_area > 200
                if MODE == "OBSTACLE"
                else False
            ):
                print("Reverse trigger detected!")
                trigger_reverse = True
                reverse_start_time = time.time()
                speed = 0  # stop before reversing
                arduino.write(f"{speed},-1,{reverse_angle}\n".encode())
                if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                print("continue")
                continue

            # --- Parking logic ---
            # if contour_workers.parking_mode:
            #     angle = parking.process_parking(
            #             parking_result=parking_result,
            #             pid=pid,
            #             left_result=left_result,
            #             right_result=right_result,
            #         )
            #     if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
            #         break

            #     continue

            # if parking_walls_count == 2:
            #     obstacle_wall_pivot = parking_wall_pivot
            #     if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
            #         break
            #     continue

            # --- Obstacle avoidance ---
            # reset obstacle positions
            # obstacle position
            red_obj_x, red_obj_y = None, None
            green_obj_x, green_obj_y = None, None
            obj_error = 0

            if contour_workers.mode == "OBSTACLE" and (
                (red_result.contours and red_area > 300)
                or (green_result.contours and green_area > 300)
            ):
                # get object coordinates
                green_obj_x, green_obj_y = get_max_y_coord(green_result.contours)
                red_obj_x, red_obj_y = get_max_y_coord(red_result.contours)

                # red_obj_x, _ = get_overall_centroid(red_result.contours)
                # green_obj_x, _ = get_overall_centroid(green_result.contours)

                # print(f"Raw Green: {green_obj_x}, {green_obj_y} | Raw Red: {red_obj_x}, {red_obj_y}")

                # set inf if no object detected
                if green_obj_y is None or green_obj_x is None:
                    green_obj_x = -1
                    green_obj_y = -1

                if red_obj_y is None or red_obj_x is None:
                    red_obj_x = -1
                    red_obj_y = -1

                # only proceed if both are not -1
                if not (
                    (green_obj_x == -1 and green_obj_y == -1)
                    and (red_obj_x == -1 and red_obj_y == -1)
                ):
                    # print(
                    #     f"Red: {red_obj_x}, {red_obj_y} | Green: {green_obj_x}, {green_obj_y}"
                    # )
                    # if object is too close, back off (convert to global coords and compare)
                    if (
                        (red_obj_y + OBS_REGION[1]) > REVERSE_REGION[1]
                        and (red_obj_x + OBS_REGION[0]) > REVERSE_REGION[0]
                        and (red_obj_x + OBS_REGION[0]) < REVERSE_REGION[2]
                    ):
                        print("Object too close! Backing off. RED Object")
                        reverse_angle = STRAIGHT_CONST - 25  # turn left when reversing
                        trigger_reverse = True
                        reverse_start_time = time.time()

                    elif (
                        (green_obj_y + OBS_REGION[1]) > REVERSE_REGION[1]
                        and (green_obj_x + OBS_REGION[0]) > REVERSE_REGION[0]
                        and (green_obj_x + OBS_REGION[0]) < REVERSE_REGION[2]
                    ):
                        print("Object too close! Backing off. GREEN Object")
                        reverse_angle = STRAIGHT_CONST + 25  # turn right when reversing
                        trigger_reverse = True
                        reverse_start_time = time.time()

                    direction_turing = ""

                    # if red obj is closer & right to the left danger zone
                    if (
                        red_obj_y > green_obj_y
                        and point_position(
                            DANGER_ZONE_POINTS[0]["x1"],
                            DANGER_ZONE_POINTS[0]["y1"],
                            DANGER_ZONE_POINTS[0]["x2"],
                            DANGER_ZONE_POINTS[0]["y2"],
                            red_obj_x + OBS_REGION[0],
                            red_obj_y + OBS_REGION[1],
                        )
                        == "RIGHT"
                    ):
                        print("Red")

                        # if front_wall_area > 350:
                        #     print("Front wall is priority, ignoring side walls")
                        #     r_wall_x = (
                        #         FRONT_WALL_REGION[0]
                        #         + (FRONT_WALL_REGION[2] - FRONT_WALL_REGION[0]) // 2
                        #     )
                        #     r_wall_y = (
                        #         FRONT_WALL_REGION[1]
                        #         + (FRONT_WALL_REGION[3] - FRONT_WALL_REGION[1]) // 2
                        #     )
                        # else:
                        r_wall_x, r_wall_y = get_overall_centroid(right_result.contours)

                        if r_wall_x is None:
                            print("No wall detected!")
                            # set default wall position if none detected
                            r_wall_x = CAM_WIDTH
                        else:
                            # transform to global coordinates
                            r_wall_x += RIGHT_REGION[0]

                        # transform to global coordinates
                        red_obj_x += OBS_REGION[0]
                        red_obj_y += OBS_REGION[1]

                        # compute how far is the bot from the object and walls middle point
                        offset_x = (red_obj_x + ((r_wall_x - red_obj_x) // 2)) - (
                            CAM_WIDTH // 2
                        )

                        # show a circle dot on the middle of the object and wall
                        obstacle_wall_pivot = (
                            red_obj_x + ((r_wall_x - red_obj_x) // 2),
                            red_obj_y,
                        )

                        obj_error = offset_x / (CAM_WIDTH // 2)  # normalized [-1, 1]
                        direction_turing = "right"

                    # if green obj is closer & left to the right danger zone
                    elif (
                        green_obj_y > red_obj_y
                        and point_position(
                            DANGER_ZONE_POINTS[1]["x1"],
                            DANGER_ZONE_POINTS[1]["y1"],
                            DANGER_ZONE_POINTS[1]["x2"],
                            DANGER_ZONE_POINTS[1]["y2"],
                            green_obj_x + OBS_REGION[0],
                            green_obj_y + OBS_REGION[1],
                        )
                        == "LEFT"
                    ):
                        print("Green")

                        # if front_wall_area > 350:
                        #     print("Front wall is priority, ignoring side walls")
                        #     l_wall_x = (
                        #         FRONT_WALL_REGION[0]
                        #         + (FRONT_WALL_REGION[2] - FRONT_WALL_REGION[0]) // 2
                        #     )
                        #     l_wall_y = (
                        #         FRONT_WALL_REGION[1]
                        #         + (FRONT_WALL_REGION[3] - FRONT_WALL_REGION[1]) // 2
                        #     )
                        # else:
                        l_wall_x, l_wall_y = get_overall_centroid(left_result.contours)

                        if l_wall_x is None:
                            print("No wall detected!")
                            # set default wall position if none detected
                            l_wall_x = 0
                        else:
                            # transform to global coordinates
                            l_wall_x += LEFT_REGION[0]

                        # transform to global coordinates
                        green_obj_x += OBS_REGION[0]
                        green_obj_y += OBS_REGION[1]

                        # compute how far is the bot from the object and walls middle point
                        offset_x = (green_obj_x - ((green_obj_x - l_wall_x) // 2)) - (
                            CAM_WIDTH // 2
                        )

                        # show a circle dot on the middle of the object and wall
                        obstacle_wall_pivot = (
                            green_obj_x - ((green_obj_x - l_wall_x) // 2),
                            green_obj_y,
                        )

                        obj_error = offset_x / (CAM_WIDTH // 2)  # normalized [-1, 1]
                        direction_turing = "left"

                    # amplify to make it more responsive
                    obj_error = -np.clip(obj_error * 7.5, -1, 1)
                    normalized_angle_offset = pid(obj_error)

                    # steer more aggressively when closer to object
                    y_gain = np.interp(
                        (
                            red_obj_y
                            if direction_turing == "right"
                            else (green_obj_y if direction_turing == "left" else 0)
                        ),
                        [
                            0,
                            (CAM_HEIGHT // 2) - 50,
                            (CAM_HEIGHT // 2) - 20,
                            OBS_REGION[3],
                        ],
                        [0, 0.45, 0.7, 1],
                    )
                    normalized_angle_offset *= y_gain
                    speed_factor = 1 - (0.3 * y_gain)  # slow down when closer to object
                    print(
                        f"Norm: {normalized_angle_offset} | Ygain: {y_gain} | OBJ error: {obj_error}"
                    )
                # print(f"Obj: {red_obj_x}, {red_obj_y} | Wall: {r_wall_x}, {r_wall_y}")
            if contour_workers.mode == "OBSTACLE":
                if (
                    front_wall_area
                    > 200
                    # and (red_area == 0 and green_area == 0)
                    # and (left_area > 800 and right_area > 800)
                ):
                    # if left_area - right_area > 1500:
                    #     normalized_angle_offset = 1  # turn right hard
                    #     print("left area too big")
                    # elif right_area - left_area > 1500:
                    #     normalized_angle_offset = -1  # turn left hard
                    #     print("right area too big")
                    # else:
                    #     normalized_angle_offset = (
                    #         -1 if parking.parking_lot_side == "left" else 1
                    #     )
                    #     print("only front wall")
                    print("+" * 50)
                    if visualizer.direction == "cw":
                        if (
                            visualizer.get_boundary_proximity(tracker.x, tracker.y)
                            # visualizer.get_boundary_vector_proximity(tracker.positions[-4][0], tracker.positions[-4][1], tracker.x, tracker.y)
                            == "close_outer"
                        ):
                            normalized_angle_offset = -1
                            print("front wall + close to outer boundary CW + left")
                        else:
                            normalized_angle_offset = 1
                            print("front wall + not close to outer boundary CW + right")
                    elif visualizer.direction == "ccw":
                        if (
                            visualizer.get_boundary_proximity(tracker.x, tracker.y)
                            # visualizer.get_boundary_vector_proximity(tracker.positions[-4][0], tracker.positions[-4][1], tracker.x, tracker.y)
                            == "close_outer"
                        ):
                            normalized_angle_offset = 1
                            print("front wall + close to outer boundary CCW + right")
                        else:
                            normalized_angle_offset = -1
                            print("front wall + not close to outer boundary CCW + left")
                    obstacle_wall_pivot = (None, None)

                    if reverse_area > 1000:
                        reverse_angle = STRAIGHT_CONST - (
                            30 * normalized_angle_offset
                        )  # turn left/right when reversing
                        trigger_reverse = True
                        reverse_start_time = time.time()

                        speed = 0  # stop before reversing
                        arduino.write(f"{speed},-1,{reverse_angle}\n".encode())
                        if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
                            break

                        print("continue")
                        continue

                #     show_front_wall = True
                # else:
                #     show_front_wall = False
                # wall following logic

            if (
                # or if x or y coords are not assigned and front area is small
                (
                    (
                        red_obj_x is None
                        and green_obj_x is None
                        and red_obj_y is None
                        and green_obj_y is None
                    )
                    and front_wall_area < 200
                )
                or (
                    # or if both obstacles are outside the danger zone
                    (
                        (
                            red_obj_x is not None
                            and red_obj_y is not None
                            and red_obj_x != -1
                            and red_obj_y != -1
                            and point_position(
                                DANGER_ZONE_POINTS[0]["x1"],
                                DANGER_ZONE_POINTS[0]["y1"],
                                DANGER_ZONE_POINTS[0]["x2"],
                                DANGER_ZONE_POINTS[0]["y2"],
                                red_obj_x + OBS_REGION[0],
                                red_obj_y + OBS_REGION[1],
                            )
                            == "LEFT"
                        )
                        or (
                            green_obj_x is not None
                            and green_obj_y is not None
                            and green_obj_x != -1
                            and green_obj_y != -1
                            and point_position(
                                DANGER_ZONE_POINTS[1]["x1"],
                                DANGER_ZONE_POINTS[1]["y1"],
                                DANGER_ZONE_POINTS[1]["x2"],
                                DANGER_ZONE_POINTS[1]["y2"],
                                green_obj_x + OBS_REGION[0],
                                green_obj_y + OBS_REGION[1],
                            )
                            == "RIGHT"
                        )
                    )
                )
            ):
                print(
                    "1st:",
                    bool(
                        (
                            (
                                red_obj_x is None
                                and green_obj_x is None
                                and red_obj_y is None
                                and green_obj_y is None
                            )
                            and front_wall_area < 200
                        )
                    ),
                )
                print(
                    "2nd: 1",
                    bool(
                        (
                            red_obj_x is not None
                            and red_obj_y is not None
                            and point_position(
                                DANGER_ZONE_POINTS[0]["x1"],
                                DANGER_ZONE_POINTS[0]["y1"],
                                DANGER_ZONE_POINTS[0]["x2"],
                                DANGER_ZONE_POINTS[0]["y2"],
                                red_obj_x + OBS_REGION[0],
                                red_obj_y + OBS_REGION[1],
                            )
                            == "LEFT"
                        )
                    ),
                )

                print(red_obj_x, red_obj_y)

                print(
                    "2nd: 2",
                    bool(
                        (
                            green_obj_x is not None
                            and green_obj_y is not None
                            and point_position(
                                DANGER_ZONE_POINTS[1]["x1"],
                                DANGER_ZONE_POINTS[1]["y1"],
                                DANGER_ZONE_POINTS[1]["x2"],
                                DANGER_ZONE_POINTS[1]["y2"],
                                green_obj_x + OBS_REGION[0],
                                green_obj_y + OBS_REGION[1],
                            )
                            == "RIGHT"
                        )
                    ),
                )

                print(green_obj_x, green_obj_y)

                obstacle_wall_pivot = (None, None)
                # PID controller
                right_total_area = (RIGHT_REGION[2] - RIGHT_REGION[0]) * (
                    RIGHT_REGION[3] - RIGHT_REGION[1]
                )
                left_total_area = (LEFT_REGION[2] - LEFT_REGION[0]) * (
                    LEFT_REGION[3] - LEFT_REGION[1]
                )
                # left_area_normalized = left_area / ((LEFT_REGION[2] - LEFT_REGION[0]) * (LEFT_REGION[3] - LEFT_REGION[1]))

                # interpolate
                right_area_normalized = np.interp(
                    right_area,
                    [0, right_total_area / 2, right_total_area],
                    [0, 0.7, 1] if MODE == "NO_OBSTACLE" else [0, 0.5, 0.7],
                )
                left_area_normalized = np.interp(
                    left_area,
                    [0, left_total_area / 2, left_total_area],
                    [0, 0.7, 1] if MODE == "NO_OBSTACLE" else [0, 0.5, 0.7],
                )

                if (
                    right_area_normalized == 0
                    and not left_area_normalized == 0
                    and MODE == "OBSTACLE"
                ):
                    # boost right side if left is detected but right is not
                    left_area_normalized = min(left_area_normalized * 2.1, 1)
                elif (
                    left_area_normalized == 0
                    and not right_area_normalized == 0
                    and MODE == "OBSTACLE"
                ):
                    # boost left side if right is detected but left is not
                    right_area_normalized = min(right_area_normalized * 2.1, 1)

                error = right_area_normalized - left_area_normalized
                # left_buf.append(left_area)
                # right_buf.append(right_area)
                # left_s = sum(left_buf) / len(left_buf)
                # right_s = sum(right_buf) / len(right_buf)
                # aDiff = right_s - left_s
                # aSum = left_s + right_s
                # error = aDiff / (aSum + 1e-6)  # normalized between roughly [-1,1]
                normalized_angle_offset = pid(error)

                print(
                    f"Wall following | Norm: {normalized_angle_offset} | Error: {error}"
                )

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
            arduino.write(f"{speed},-1,{angle}\n".encode())

            # intersection detection
            # work here
            # if not intersection_detected:
            #     if (
            #         (orange_result.contours and orange_area > 80)
            #         or (blue_result.contours and blue_area > 80)
            #     ) and (
            #         # ignore laps if just reversed recently for 1.4 second
            #         reverse_start_time + reverse_duration + 1.4
            #         < time.time()
            #     ):
            #         intersection_detected = True
            #         intersection_crossing_start = int(time.time())
            #         current_intersections += 1

            #         # start a thread to buzz for 0.3s
            #         threading.Thread(
            #             target=lambda: (
            #                 GPIO.output(BUZZER_PIN, GPIO.HIGH),
            #                 time.sleep(0.3),
            #                 GPIO.output(BUZZER_PIN, GPIO.LOW),
            #             ),
            #             daemon=True,
            #         ).start()

            #         print(
            #             f"Intersection detected! Count: {current_intersections}/{TOTAL_INTERSECTIONS}"
            #         )
            # else:
            #     if (
            #         int(time.time()) - intersection_crossing_start
            #         > intersection_crossing_duration
            #     ):
            #         intersection_detected = False
            #         print("Intersection crossing ended.")
            #     else:
            #         if (orange_result.contours and orange_area > 80) or (
            #             blue_result.contours and blue_area > 80
            #         ):
            #             # keep latest time to avoid multiple increments
            #             intersection_crossing_start = int(time.time())

            # Stopping logic
            if (
                # contour_workers.mode == "NO_OBSTACLE"
                # and
                stopFlag
                and (clamp_angle(gyro_angle, threshold=50) % 360 == 0)
                # and (int(time.time()) - stopTime) > stop_timer
            ):
                print("Lap completed!")
                arduino.write(f"-5,-1,{angle}\n".encode())
                print(angle)
                break

            if current_intersections >= TOTAL_INTERSECTIONS and not stopFlag:
                stopFlag = True
                # Enable parking mode if in obstacle mode
                contour_workers.parking_mode = True if MODE == "OBSTACLE" else False
                stopTime = int(time.time())
                print("Preparing to stop...")

            if DEBUG and cv2.waitKey(1) & 0xFF == ord("q"):
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
        tracker.close()


if __name__ == "__main__":
    main()

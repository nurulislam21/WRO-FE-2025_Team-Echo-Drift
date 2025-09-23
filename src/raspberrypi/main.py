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
)
from contour_workers import ContourWorkers
from parking import Parking
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
MIN_SPEED = 50

# Intersections
TOTAL_INTERSECTIONS = 12

# Region of Interest coordinates
LEFT_REGION = [20, 220, 270, 280]  # left
RIGHT_REGION = [370, 220, 620, 280]  # right
LAP_REGION = [200, 300, 440, 350]  # lap detection
OBS_REGION = [95, 140, 545, 320]  # obstacle detection
REVERSE_REGION = [200, 300, 440, 320]  # reverse trigger area
FRONT_WALL_REGION = [300, 200, 340, 220]  # front wall detection
PARKING_LOT_REGION = [0, 185, CAM_WIDTH, 400]  # parking lot detection

BLACK_WALL_DETECTOR_AREA = (LEFT_REGION[2] - LEFT_REGION[0]) * (
    LEFT_REGION[3] - LEFT_REGION[1]
)
OBSTACLE_DETECTOR_X = OBS_REGION[2] - OBS_REGION[0]
OBSTACLE_DETECTOR_Y = OBS_REGION[3] - OBS_REGION[1]
obstacle_wall_pivot = (None, None)

# Color ranges
LOWER_BLACK = np.array([0, 111, 116])
UPPER_BLACK = np.array([80, 151, 156])

LOWER_ORANGE = np.array([105, 125, 87])
UPPER_ORANGE = np.array([185, 165, 127])

LOWER_BLUE = np.array([92, 150, 166])
UPPER_BLUE = np.array([152, 190, 206])

# obstacle color ranges
LOWER_RED = np.array([50, 163, 42])
UPPER_RED = np.array([120, 203, 82])

LOWER_GREEN = np.array([95, 96, 165])
UPPER_GREEN = np.array([180, 136, 205])

# parking color ranges
LOWER_MAGENTA = np.array([90, 89, 105])
UPPER_MAGENTA = np.array([160, 129, 145])


contour_workers = ContourWorkers(
    #mode="NO_OBSTACLE",
    mode="OBSTACLE",
    has_parked_out=False,
    # color ranges
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


MAX_OFFSET_DEGREE = 40
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
reverse_duration = 0.6  # seconds

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
time.sleep(2)
arduino.write(b"0,-1,95\n")

# parking
parking = Parking(
    parking_speed=45,
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


# Threading variables - separate queues for each detection task
def main():
    global stopFlag, stopTime, speed, trigger_reverse
    global current_intersections, intersection_detected, intersection_crossing_start
    global startProcessing, obstacle_wall_pivot

    parking_walls = []
    parking_walls_count = 0
    parking_wall_pivot = (None, None)

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
        contour_workers.reverse_n_front_wall_contour_worker,
        contour_workers.parking_lot_contour_worker,
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
            if not startProcessing and arduino.in_waiting > 0:
                line = arduino.readline().decode("utf-8").rstrip()
                print(f"Arduino: {line}")
                if line == "START":
                    startProcessing = True

            # Capture frame
            frame = picam2.capture_array()

            # Distribute frame to all processing threads (non-blocking)
            frame_copy = copy.deepcopy(frame)
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
                    angle=angle,
                    current_intersections=current_intersections,
                    left_area=left_area,
                    right_area=right_area,
                    obstacle_wall_pivot=obstacle_wall_pivot,
                    parking_mode=contour_workers.parking_mode,
                    parking_lot_region=PARKING_LOT_REGION,
                    parking_result=parking_result,
                )

            if not startProcessing:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                # Dont start moving until start signal received
                continue

            # --- PARKING LOGIC ---
            if contour_workers.mode == "OBSTACLE":
                # process parking out first if not yet done
                if not parking.has_parked_out:
                    parking.process_parking_out(
                        left_result=left_result, right_result=right_result
                    )
                    parking.has_parked_out = contour_workers.has_parked_out = True
                    continue

                # process parking, when parking mode is active
                if contour_workers.parking_mode:
                    angle = parking.process_parking(
                        parking_result=parking_result,
                        pid=pid,
                        left_result=left_result,
                        right_result=right_result,
                    )
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    continue

            # --- Reversing logic ---
            if trigger_reverse:
                speed = -MIN_SPEED
                if (time.time() - reverse_start_time) > reverse_duration:
                    trigger_reverse = False
                    speed = 0  # stop after reversing
                else:
                    angle = STRAIGHT_CONST  # go straight when reversing
                arduino.write(f"{speed},-1,{angle}\n".encode())
                print(f"Reversing... Speed: {speed}, Angle: {angle}")
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                print("continue")
                continue

            elif reverse_area > 1500:
                print("Reverse trigger detected!")
                trigger_reverse = True
                reverse_start_time = time.time()
                speed = 0  # stop before reversing
                arduino.write(f"{speed},-1,{angle}\n".encode())
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                print("continue")
                continue

            # if parking_walls_count == 2:
            #     obstacle_wall_pivot = parking_wall_pivot
            #     if cv2.waitKey(1) & 0xFF == ord("q"):
            #         break
            #     continue

            # --- Obstacle avoidance ---
            if contour_workers.mode == "OBSTACLE" and (
                (red_result.contours and red_area > 300)
                or (green_result.contours and green_area > 300)
            ):
                # get object coordinates
                green_obj_x, green_obj_y = get_max_y_coord(green_result.contours)
                red_obj_x, red_obj_y = get_max_y_coord(red_result.contours)

                # set inf if no object detected
                if green_obj_y is None or green_obj_x is None:
                    green_obj_x = -1
                    green_obj_y = -1

                if red_obj_y is None or red_obj_x is None:
                    red_obj_x = -1
                    red_obj_y = -1

                # only proceed if both are not infinity
                if not (
                    (green_obj_x == -1 and green_obj_y == -1)
                    and (red_obj_x == -1 and red_obj_y == -1)
                ):

                    print(
                        f"Red: {red_obj_x}, {red_obj_y} | Green: {green_obj_x}, {green_obj_y}"
                    )
                    # if object is too close, back off (convert to global coords and compare)
                    if (
                        (red_obj_y + OBS_REGION[1]) > REVERSE_REGION[1]
                        and (red_obj_x + OBS_REGION[0]) > REVERSE_REGION[0]
                        and (red_obj_x + OBS_REGION[0]) < REVERSE_REGION[2]
                    ) or (
                        (green_obj_y + OBS_REGION[1]) > REVERSE_REGION[1]
                        and (green_obj_x + OBS_REGION[0]) > REVERSE_REGION[0]
                        and (green_obj_x + OBS_REGION[0]) < REVERSE_REGION[2]
                    ):
                        print("Object too close! Backing off.")
                        trigger_reverse = True
                        reverse_start_time = time.time()

                    direction_turing = ""

                    # if red obj is closer
                    if red_obj_y > green_obj_y:
                        if front_wall_area > 500:
                            print("Front wall is priority, ignoring side walls")
                            r_wall_x = (
                                FRONT_WALL_REGION[0]
                                + (FRONT_WALL_REGION[2] - FRONT_WALL_REGION[0]) // 2
                            )
                            r_wall_y = (
                                FRONT_WALL_REGION[1]
                                + (FRONT_WALL_REGION[3] - FRONT_WALL_REGION[1]) // 2
                            )
                        else:
                            r_wall_x, r_wall_y = get_overall_centroid(
                                right_result.contours
                            )

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

                    # if green obj is closer
                    elif green_obj_y > red_obj_y:
                        if front_wall_area > 500:
                            print("Front wall is priority, ignoring side walls")
                            l_wall_x = (
                                FRONT_WALL_REGION[0]
                                + (FRONT_WALL_REGION[2] - FRONT_WALL_REGION[0]) // 2
                            )
                            l_wall_y = (
                                FRONT_WALL_REGION[1]
                                + (FRONT_WALL_REGION[3] - FRONT_WALL_REGION[1]) // 2
                            )
                        else:
                            l_wall_x, l_wall_y = get_overall_centroid(
                                left_result.contours
                            )

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
                        [0, 0.2, 0.7, 1],
                    )
                    normalized_angle_offset *= y_gain
                    speed_factor = 1 - (0.3 * y_gain)  # slow down when closer to object
                    print(
                        f"Norm: {normalized_angle_offset} | Ygain: {y_gain} | OBJ error: {obj_error}"
                    )

                # print(f"Obj: {red_obj_x}, {red_obj_y} | Wall: {r_wall_x}, {r_wall_y}")

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
            if (
                contour_workers.mode == "NO_OBSTACLE"
                and stopFlag
                and (int(time.time()) - stopTime) > 1.7
            ):
                print("Lap completed!")
                arduino.write(f"-5,-1,{angle}\n".encode())
                print(angle)
                break

            if (
                current_intersections >= TOTAL_INTERSECTIONS
                # and abs(angle - STRAIGHT_CONST) <= 15
                and not stopFlag
            ):
                stopFlag = True
                contour_workers.parking_mode = True
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

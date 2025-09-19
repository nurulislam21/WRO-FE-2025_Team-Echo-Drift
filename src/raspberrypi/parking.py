import cv2
from serial import Serial
from contour_workers import ContourResult
from simple_pid import PID
import numpy as np
import time


class Parking:
    def __init__(
        self,
        arduino: Serial,
        parking_speed: int,
        camera_width: int,
        camera_height: int,
        parking_lot_region: list,
        maxLeft: int,
        maxRight: int,
        STRAIGHT_CONST: int,
        MAX_OFFSET_DEGREE: int,
        REVERSE_REGION: list,
    ):
        self.arduino = arduino
        self.parking_speed = parking_speed
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.parking_lot_region = parking_lot_region

        # steering constraints
        self.maxLeft = maxLeft
        self.maxRight = maxRight
        self.STRAIGHT_CONST = STRAIGHT_CONST
        self.MAX_OFFSET_DEGREE = MAX_OFFSET_DEGREE

        # state
        self.last_wall_count = 0
        self.stop_tolerance = 5

        # reverse
        self.REVERSE_REGION = REVERSE_REGION
        self.reverse_start_time = 0
        self.reverse_duration = 0.2  # seconds
        self.is_reversing = False

    def process_parking(self, parking_result: ContourResult, pid: PID):
        parking_walls = []
        current_wall_count = 0
        obstacle_wall_pivot = (None, None)

        if parking_result and parking_result.area > 800:
            for contour in parking_result.contours:
                x, y, w, h = cv2.boundingRect(contour)
                area = w * h

                if area > 1500:
                    current_wall_count += 1
                    parking_walls.append((x, y, w, h))

        if self.last_wall_count == 2 and current_wall_count < 2:
            self.is_reversing = True
            self.reverse_start_time = time.time()

        if self.is_reversing:
            if (time.time() - self.reverse_start_time) < self.reverse_duration:
                self.arduino.write(
                    f"{-self.parking_speed}, {self.STRAIGHT_CONST}\n".encode()
                )
                return parking_walls, 2, obstacle_wall_pivot
            else:
                self.is_reversing = False
                self.last_wall_count = 0

        self.last_wall_count = current_wall_count
        if current_wall_count == 2:
            wall_1_x, wall_1_y, wall_1_w, wall_1_h = parking_walls[0]
            wall_1_cx, wall_1_cy = wall_1_x + (wall_1_w // 2), wall_1_y + (
                wall_1_h // 2
            )
            wall_2_x, wall_2_y, wall_2_w, wall_2_h = parking_walls[1]
            wall_2_cx, wall_2_cy = wall_2_x + (wall_2_w // 2), wall_2_y + (
                wall_2_h // 2
            )

            # check if the centroid y are in the reverse region
            if (
                ((wall_1_y + self.parking_lot_region[1] + wall_1_h) > self.REVERSE_REGION[1])
                and ((wall_1_cx + self.parking_lot_region[0]) > self.REVERSE_REGION[0])
                and ((wall_1_cx + self.parking_lot_region[0]) < self.REVERSE_REGION[2])
            ) or (
                ((wall_2_y + self.parking_lot_region[1] + wall_2_h) > self.REVERSE_REGION[1])
                and ((wall_2_cx + self.parking_lot_region[0]) > self.REVERSE_REGION[0])
                and ((wall_2_cx + self.parking_lot_region[0]) < self.REVERSE_REGION[2])
            ):
                print("Entering Reverse Region")
                self.is_reversing = True
                self.reverse_start_time = time.time()
                return parking_walls, 2, obstacle_wall_pivot

            print(
                f"wall 1: X: {wall_1_cx + self.parking_lot_region[0]}, Y: {wall_1_cy + self.parking_lot_region[1] + wall_1_h} | REVERSE_REGION: {self.REVERSE_REGION}"
            )
            print(
                f"wall 2: X: {wall_2_cx + self.parking_lot_region[0]}, Y: {wall_2_cy + self.parking_lot_region[1] + wall_2_h} | REVERSE_REGION: {self.REVERSE_REGION}"
            )
            # print(
            #     f"wall 2: area: {wall_2_w * wall_2_h}, pos: ({wall_2_cx}, {wall_2_cy})"
            # )

            # transform to global coordinates
            wall_1_cx += self.parking_lot_region[0]
            wall_1_cy += self.parking_lot_region[1]
            wall_2_cx += self.parking_lot_region[0]
            wall_2_cy += self.parking_lot_region[1]

            # compute how far is the bot from the object and walls middle point
            offset_x = ((wall_1_cx + wall_2_cx) // 2) - (self.camera_width // 2)

            # show a circle dot on the middle of the object and wall
            obstacle_wall_pivot = (
                (wall_1_cx + wall_2_cx) // 2,
                (wall_1_cy + wall_2_cy) // 2,
            )

            obj_error = offset_x / (self.camera_width // 2)
            obj_error = -np.clip(obj_error * 10, -1, 1)
            normalized_angle_offset = pid(obj_error)

            # --- Map normalized control to servo angle ---
            angle = int(
                max(
                    min(
                        self.STRAIGHT_CONST
                        + normalized_angle_offset * self.MAX_OFFSET_DEGREE,
                        self.maxRight,
                    ),
                    self.maxLeft,
                )
            )

            self.arduino.write(f"{self.parking_speed}, {angle}\n".encode())

            # if abs(offset_x) < self.stop_tolerance:
            #     print("Parking | Stopped")
            #     print(f"offset_x: {offset_x}, obj_error: {obj_error}, angle: {angle}")
            #     while True:
            #         self.arduino.write(f"0, {self.STRAIGHT_CONST}\n".encode())
            #         time.sleep(0.1)

            # print(f"Parking | Speed: {self.parking_speed}, Angle: {angle}")

        return parking_walls, current_wall_count, obstacle_wall_pivot

import cv2
from serial import Serial
from contour_workers import ContourResult
from simple_pid import PID
import numpy as np


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


    def process_parking(self, parking_result: ContourResult, pid: PID):
        parking_walls = []
        parking_wall_count = 0
        obstacle_wall_pivot = (None, None)

        if parking_result and parking_result.area > 800:
            for contour in parking_result.contours:
                x, y, w, h = cv2.boundingRect(contour)
                area = w * h

                if area > 1000:
                    parking_wall_count += 1
                    parking_walls.append((x, y, w, h))

        if parking_wall_count == 2:
            wall_1_x, wall_1_y, _, _ = parking_walls[0]
            wall_2_x, wall_2_y, _, _ = parking_walls[1]

            # transform to global coordinates
            wall_1_x += self.parking_lot_region[0]
            wall_1_y += self.parking_lot_region[1]
            wall_2_x += self.parking_lot_region[0]
            wall_2_y += self.parking_lot_region[1]

            # compute how far is the bot from the object and walls middle point
            offset_x = ((wall_1_x + wall_2_x) // 2) - (self.camera_width // 2)

            # show a circle dot on the middle of the object and wall
            obstacle_wall_pivot = (
                (wall_1_x + wall_2_x) // 2,
                (wall_1_y + wall_2_y) // 2,
            )

            obj_error = offset_x / (self.camera_width // 2)
            obj_error = -np.clip(obj_error * 7.5, -1, 1)
            normalized_angle_offset = pid(obj_error)

            # --- Map normalized control to servo angle ---
            angle = int(
                max(
                    min(
                        self.STRAIGHT_CONST + normalized_angle_offset * self.MAX_OFFSET_DEGREE,
                        self.maxRight,
                    ),
                    self.maxLeft,
                )
            )

            self.arduino.write(f"{self.parking_speed}, {angle}\n".encode())
            print(f"Parking | Speed: {self.parking_speed}, Angle: {angle}")

        return parking_walls, parking_wall_count, obstacle_wall_pivot
